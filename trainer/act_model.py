"""
act_model.py — ACT (Action Chunking with Transformers, Zhao et al. 2023) 순수 PyTorch 구현

구조·기본값은 LeRobot policies/act/modeling_act.py 와 같게 맞춤 (CVAE 인코더 + ResNet18 + Transformer 인코더/디코더,
디코더 1 층, post-norm, ImageNet 정규화, L1 + KL 가중 10). LeRobot 패키지를 쓰지 않는 이유는 README 참고
(버전 고정·RTX 5090 용 torch 와의 충돌·OpenVINO 변환 단순화).

  ACT        : 학습용 (forward(img, state, actions, is_pad) → 예측, (mu, logvar))
  ACTExport  : 추론용 래퍼 — 입력 image [B,3,H,W] 0~1 RGB, state [B,D] rad → action [B,chunk,D] rad
               (정규화 / 역정규화를 모델 안에 넣어 로봇 쪽은 OpenVINO infer 한 번이면 끝)
"""
import math
from dataclasses import asdict, dataclass

import numpy as np
import torch
import torch.nn.functional as F
from torch import nn

IMAGENET_MEAN = [0.485, 0.456, 0.406]
IMAGENET_STD = [0.229, 0.224, 0.225]


@dataclass
class ACTConfig:
    state_dim: int = 14
    action_dim: int = 14
    image_h: int = 240
    image_w: int = 320
    chunk_size: int = 100
    dim_model: int = 512
    n_heads: int = 8
    dim_feedforward: int = 3200
    n_encoder_layers: int = 4
    n_decoder_layers: int = 1          # LeRobot 과 같음 (원 구현의 7 층은 버그로 1 층만 쓰였음)
    use_vae: bool = True
    latent_dim: int = 32
    n_vae_encoder_layers: int = 4
    dropout: float = 0.1
    kl_weight: float = 10.0
    pre_norm: bool = False
    pretrained_backbone: bool = True   # torchvision ResNet18 ImageNet 가중치 (처음 한 번 내려받음)

    def to_dict(self):
        return asdict(self)


def sinusoid_1d(num_positions, dimension):
    pos = np.arange(num_positions)[:, None]
    j = np.arange(dimension)[None, :]
    table = pos / np.power(10000, 2 * (j // 2) / dimension)
    table[:, 0::2] = np.sin(table[:, 0::2])
    table[:, 1::2] = np.cos(table[:, 1::2])
    return torch.from_numpy(table).float()


def sinusoid_2d(h, w, dimension, temperature=10000.0):
    """LeRobot ACTSinusoidalPositionEmbedding2d 와 같은 값 → (1, 2·dimension, h, w)."""
    eps, two_pi = 1e-6, 2 * math.pi
    ones = torch.ones(1, h, w)
    y = ones.cumsum(1)
    x = ones.cumsum(2)
    y = y / (y[:, -1:, :] + eps) * two_pi
    x = x / (x[:, :, -1:] + eps) * two_pi
    inv = temperature ** (2 * (torch.arange(dimension, dtype=torch.float32) // 2) / dimension)
    x = x.unsqueeze(-1) / inv
    y = y.unsqueeze(-1) / inv
    px = torch.stack((x[..., 0::2].sin(), x[..., 1::2].cos()), dim=-1).flatten(3)
    py = torch.stack((y[..., 0::2].sin(), y[..., 1::2].cos()), dim=-1).flatten(3)
    return torch.cat((py, px), dim=3).permute(0, 3, 1, 2)


class EncoderLayer(nn.Module):
    def __init__(self, c: ACTConfig):
        super().__init__()
        self.self_attn = nn.MultiheadAttention(c.dim_model, c.n_heads, dropout=c.dropout)
        self.linear1 = nn.Linear(c.dim_model, c.dim_feedforward)
        self.linear2 = nn.Linear(c.dim_feedforward, c.dim_model)
        self.norm1 = nn.LayerNorm(c.dim_model)
        self.norm2 = nn.LayerNorm(c.dim_model)
        self.dropout = nn.Dropout(c.dropout)
        self.dropout1 = nn.Dropout(c.dropout)
        self.dropout2 = nn.Dropout(c.dropout)
        self.pre_norm = c.pre_norm

    def forward(self, x, pos=None, key_padding_mask=None):
        skip = x
        if self.pre_norm:
            x = self.norm1(x)
        q = k = x if pos is None else x + pos
        x = self.self_attn(q, k, value=x, key_padding_mask=key_padding_mask, need_weights=False)[0]
        x = skip + self.dropout1(x)
        if self.pre_norm:
            skip = x
            x = self.norm2(x)
        else:
            x = self.norm1(x)
            skip = x
        x = self.linear2(self.dropout(F.relu(self.linear1(x))))
        x = skip + self.dropout2(x)
        if not self.pre_norm:
            x = self.norm2(x)
        return x


class Encoder(nn.Module):
    def __init__(self, c: ACTConfig, n_layers):
        super().__init__()
        self.layers = nn.ModuleList([EncoderLayer(c) for _ in range(n_layers)])
        self.norm = nn.LayerNorm(c.dim_model) if c.pre_norm else nn.Identity()

    def forward(self, x, pos=None, key_padding_mask=None):
        for layer in self.layers:
            x = layer(x, pos, key_padding_mask)
        return self.norm(x)


class DecoderLayer(nn.Module):
    def __init__(self, c: ACTConfig):
        super().__init__()
        self.self_attn = nn.MultiheadAttention(c.dim_model, c.n_heads, dropout=c.dropout)
        self.multihead_attn = nn.MultiheadAttention(c.dim_model, c.n_heads, dropout=c.dropout)
        self.linear1 = nn.Linear(c.dim_model, c.dim_feedforward)
        self.linear2 = nn.Linear(c.dim_feedforward, c.dim_model)
        self.norm1 = nn.LayerNorm(c.dim_model)
        self.norm2 = nn.LayerNorm(c.dim_model)
        self.norm3 = nn.LayerNorm(c.dim_model)
        self.dropout = nn.Dropout(c.dropout)
        self.dropout1 = nn.Dropout(c.dropout)
        self.dropout2 = nn.Dropout(c.dropout)
        self.dropout3 = nn.Dropout(c.dropout)
        self.pre_norm = c.pre_norm

    def forward(self, x, enc, dec_pos, enc_pos):
        skip = x
        if self.pre_norm:
            x = self.norm1(x)
        q = k = x + dec_pos
        x = self.self_attn(q, k, value=x, need_weights=False)[0]
        x = skip + self.dropout1(x)
        if self.pre_norm:
            skip = x
            x = self.norm2(x)
        else:
            x = self.norm1(x)
            skip = x
        x = self.multihead_attn(query=x + dec_pos, key=enc + enc_pos, value=enc, need_weights=False)[0]
        x = skip + self.dropout2(x)
        if self.pre_norm:
            skip = x
            x = self.norm3(x)
        else:
            x = self.norm2(x)
            skip = x
        x = self.linear2(self.dropout(F.relu(self.linear1(x))))
        x = skip + self.dropout3(x)
        if not self.pre_norm:
            x = self.norm3(x)
        return x


class Decoder(nn.Module):
    def __init__(self, c: ACTConfig):
        super().__init__()
        self.layers = nn.ModuleList([DecoderLayer(c) for _ in range(c.n_decoder_layers)])
        self.norm = nn.LayerNorm(c.dim_model)

    def forward(self, x, enc, dec_pos, enc_pos):
        for layer in self.layers:
            x = layer(x, enc, dec_pos, enc_pos)
        return self.norm(x)


def make_backbone(pretrained):
    """ResNet18 layer4 까지 (FrozenBatchNorm2d — LeRobot 과 같음). 가중치를 못 받으면 무작위 초기화 + 경고."""
    import torchvision
    from torchvision.ops.misc import FrozenBatchNorm2d
    weights, note = None, "무작위 초기화"
    if pretrained:
        try:
            weights = torchvision.models.ResNet18_Weights.IMAGENET1K_V1
            net = torchvision.models.resnet18(weights=weights, norm_layer=FrozenBatchNorm2d)
            note = "ImageNet 사전학습"
        except Exception as e:   # noqa: BLE001 — 오프라인 등
            print(f"[act] ⚠ ResNet18 ImageNet 가중치를 못 받음 ({e}) — 무작위 초기화로 학습 (성능 낮을 수 있음)", flush=True)
            net = torchvision.models.resnet18(weights=None, norm_layer=FrozenBatchNorm2d)
    else:
        net = torchvision.models.resnet18(weights=None, norm_layer=FrozenBatchNorm2d)
    body = nn.Sequential(net.conv1, net.bn1, net.relu, net.maxpool, net.layer1, net.layer2, net.layer3, net.layer4)
    return body, note


class ACT(nn.Module):
    def __init__(self, c: ACTConfig):
        super().__init__()
        self.c = c
        D = c.dim_model
        if c.use_vae:
            self.vae_encoder = Encoder(c, c.n_vae_encoder_layers)
            self.vae_encoder_cls_embed = nn.Embedding(1, D)
            self.vae_encoder_robot_state_input_proj = nn.Linear(c.state_dim, D)
            self.vae_encoder_action_input_proj = nn.Linear(c.action_dim, D)
            self.vae_encoder_latent_output_proj = nn.Linear(D, c.latent_dim * 2)
            self.register_buffer("vae_encoder_pos_enc", sinusoid_1d(2 + c.chunk_size, D).unsqueeze(0))
        self.backbone, self.backbone_note = make_backbone(c.pretrained_backbone)
        self.encoder = Encoder(c, c.n_encoder_layers)
        self.decoder = Decoder(c)
        self.encoder_robot_state_input_proj = nn.Linear(c.state_dim, D)
        self.encoder_latent_input_proj = nn.Linear(c.latent_dim, D)
        self.encoder_img_feat_input_proj = nn.Conv2d(512, D, kernel_size=1)
        self.encoder_1d_feature_pos_embed = nn.Embedding(2, D)          # latent, state
        self.decoder_pos_embed = nn.Embedding(c.chunk_size, D)
        self.action_head = nn.Linear(D, c.action_dim)
        # 특징맵 크기는 입력 크기로 정해짐 → 2D 위치 임베딩을 상수로
        with torch.no_grad():
            fh, fw = self.backbone(torch.zeros(1, 3, c.image_h, c.image_w)).shape[-2:]
        self.feat_hw = (int(fh), int(fw))
        self.register_buffer("cam_pos", sinusoid_2d(self.feat_hw[0], self.feat_hw[1], D // 2))
        for m in (self.encoder, self.decoder):
            for p in m.parameters():
                if p.dim() > 1:
                    nn.init.xavier_uniform_(p)

    def forward(self, img, state, actions=None, is_pad=None):
        """img (B,3,H,W) 정규화됨, state (B,S), actions (B,chunk,A) 정규화됨 (학습 때만), is_pad (B,chunk) bool."""
        c = self.c
        B = state.shape[0]
        mu = logvar = None
        if c.use_vae and actions is not None:
            cls = self.vae_encoder_cls_embed.weight.unsqueeze(0).expand(B, -1, -1)
            st = self.vae_encoder_robot_state_input_proj(state).unsqueeze(1)
            ac = self.vae_encoder_action_input_proj(actions)
            x = torch.cat([cls, st, ac], dim=1).permute(1, 0, 2)
            pos = self.vae_encoder_pos_enc.permute(1, 0, 2)
            pad = torch.cat([torch.zeros(B, 2, dtype=torch.bool, device=state.device), is_pad], dim=1)
            out = self.vae_encoder(x, pos, key_padding_mask=pad)[0]
            p = self.vae_encoder_latent_output_proj(out)
            mu, logvar = p[:, :c.latent_dim], p[:, c.latent_dim:]
            latent = mu + (logvar / 2).exp() * torch.randn_like(mu)
        else:
            latent = torch.zeros(B, c.latent_dim, dtype=state.dtype, device=state.device)

        feat = self.encoder_img_feat_input_proj(self.backbone(img))            # (B, D, h, w)
        feat = feat.flatten(2).permute(2, 0, 1)                                 # (hw, B, D)
        cam_pos = self.cam_pos.to(feat.dtype).flatten(2).permute(2, 0, 1).expand(-1, B, -1)
        tok = torch.stack([self.encoder_latent_input_proj(latent),
                           self.encoder_robot_state_input_proj(state)], dim=0)  # (2, B, D)
        pos1 = self.encoder_1d_feature_pos_embed.weight.unsqueeze(1).expand(-1, B, -1)
        x = torch.cat([tok, feat], dim=0)
        pos = torch.cat([pos1, cam_pos], dim=0)
        enc = self.encoder(x, pos)
        dec_in = torch.zeros(c.chunk_size, B, c.dim_model, dtype=enc.dtype, device=enc.device)
        dec_pos = self.decoder_pos_embed.weight.unsqueeze(1).expand(-1, B, -1)
        dec = self.decoder(dec_in, enc, dec_pos, pos)
        return self.action_head(dec.transpose(0, 1)), (mu, logvar)

    def loss(self, img, state, actions, is_pad):
        pred, (mu, logvar) = self(img, state, actions, is_pad)
        l1 = (F.l1_loss(pred, actions, reduction="none") * (~is_pad).unsqueeze(-1)).mean()
        if mu is not None:
            kld = (-0.5 * (1 + logvar - mu.pow(2) - logvar.exp())).sum(-1).mean()
            total = l1 + self.c.kl_weight * kld
        else:
            kld = torch.zeros((), device=l1.device)
            total = l1
        return total, l1, kld


class ACTExport(nn.Module):
    """로봇 추론용 — 정규화 포함. 입력 image (B,3,H,W) 0~1 RGB, state (B,S) rad → action (B,chunk,A) rad."""

    def __init__(self, act: ACT, stats):
        super().__init__()
        self.act = act
        f = lambda k: torch.tensor(np.asarray(stats[k], dtype=np.float32))
        self.register_buffer("img_mean", torch.tensor(IMAGENET_MEAN).view(1, 3, 1, 1))
        self.register_buffer("img_std", torch.tensor(IMAGENET_STD).view(1, 3, 1, 1))
        self.register_buffer("s_mean", f("state_mean"))
        self.register_buffer("s_std", f("state_std"))
        self.register_buffer("a_mean", f("action_mean"))
        self.register_buffer("a_std", f("action_std"))

    def forward(self, image, state):
        x = (image - self.img_mean) / self.img_std
        s = (state - self.s_mean) / self.s_std
        a, _ = self.act(x, s)
        return a * self.a_std + self.a_mean
