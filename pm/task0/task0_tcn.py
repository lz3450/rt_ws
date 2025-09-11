#!/usr/bin/env python
#
# task0_tcn.py
#

import math
import numpy as np
import pandas as pd
from typing import Sequence, Tuple, Optional

import torch
import torch.nn as nn
from torch.utils.data import Dataset, DataLoader

# Optional but recommended
from sklearn.preprocessing import StandardScaler


# ----------------------------
# 1) Config
# ----------------------------
ALL_COLUMNS = [
    "joint1_vel",
    "joint2_vel",
    "joint3_vel",
    "joint4_vel",
    "joint5_vel",
    "joint6_vel",
    "joint7_vel",
    "joint1_eff",
    "joint2_eff",
    "joint3_eff",
    "joint4_eff",
    "joint5_eff",
    "joint6_eff",
    "joint7_eff",
    "dt",
    "joint1_err",
    "joint2_err",
    "joint3_err",
    "joint4_err",
    "joint5_err",
    "joint6_err",
    "joint7_err",
]
TARGET_COLUMNS = [
    "joint1_eff",
    "joint2_eff",
    "joint3_eff",
    "joint4_eff",
    "joint5_eff",
    "joint6_eff",
    "joint7_eff",
]


# Modern device setup
def get_device() -> torch.device:
    """Get the best available device with proper error handling."""
    if torch.cuda.is_available():
        return torch.device("cuda")
    elif hasattr(torch.backends, "mps") and torch.backends.mps.is_available():
        return torch.device("mps")  # Apple Silicon support
    else:
        return torch.device("cpu")


DEVICE = get_device()

# Training configuration - improved defaults
WINDOW = 1  # lookback window length (>= model receptive field)
BATCH_SIZE = 2
LR = 1e-3
EPOCHS = 100
PATIENCE = 10  # early stopping patience
DROPOUT = 0.1
CHANNELS = 64  # hidden channels in TCN
KERNEL = 3
DILATIONS = (1, 2, 4, 8, 16)  # grows receptive field logarithmically
WEIGHT_DECAY = 1e-5  # L2 regularization
GRAD_CLIP_NORM = 1.0  # gradient clipping


# ----------------------------
# 2) Dataset: sliding windows
# ----------------------------
class WindowedNextStepDataset(Dataset):
    """
    Builds (X_{t-W+1:t}, y_{t+1}) pairs.
    X: all features (F) over WINDOW timesteps
    y: next-step targets (7-dim efforts)
    """

    def __init__(self, data_arr: np.ndarray, target_idx: Sequence[int], window: int):
        """
        data_arr: shape (N, F) standardized features
        target_idx: indices of target columns in data_arr
        window: window length W
        """
        self.X = torch.from_numpy(data_arr).float()  # Convert once to avoid repeated conversions
        self.target_idx = torch.tensor(target_idx, dtype=torch.long)
        self.W = window
        self.N, self.F = data_arr.shape
        # We can go up to N - W - 1 (because we predict t+1)
        self.max_start = self.N - self.W - 1
        if self.max_start <= 0:
            raise ValueError(
                f"Not enough data for window size {window}. Need at least {window + 1} samples, got {self.N}."
            )

    def __len__(self) -> int:
        return self.max_start + 1

    def __getitem__(self, i: int) -> Tuple[torch.Tensor, torch.Tensor]:
        # window covers [i, i+W-1], target is at (i+W) for next-step
        x_win = self.X[i : i + self.W, :]  # (W, F)
        y_next = self.X[i + self.W, self.target_idx]  # (7,)
        return x_win, y_next


def collate_to_channels_last(batch: Sequence[Tuple[torch.Tensor, torch.Tensor]]) -> Tuple[torch.Tensor, torch.Tensor]:
    """Optimized collate function with better type hints."""
    X_list, y_list = zip(*batch)
    X = torch.stack(X_list, dim=0)  # (B, W, F)
    X = X.transpose(1, 2)  # → (B, F, W) for Conv1d
    y = torch.stack(y_list, dim=0)  # (B, 7)
    return X, y


# ----------------------------
# 3) TCN building blocks
# ----------------------------
class CausalConv1dSame(nn.Conv1d):
    """
    1D conv with padding to preserve length for causal convolution.
    We pad only on the left to avoid "future" leakage.
    """

    def __init__(self, in_channels: int, out_channels: int, kernel_size: int, dilation: int = 1):
        super().__init__(in_channels, out_channels, kernel_size, padding=0, dilation=dilation)
        self.left_pad = (kernel_size - 1) * dilation

    def forward(self, input: torch.Tensor) -> torch.Tensor:
        # Pad on the left: (left, right)
        x = nn.functional.pad(input, (self.left_pad, 0))
        return super().forward(x)


class TCNBlock(nn.Module):
    """Optimized TCN block with modern best practices."""

    def __init__(self, channels: int, kernel_size: int, dilation: int, p_drop: float):
        super().__init__()
        # Modern weight normalization API
        self.conv1 = nn.utils.parametrizations.weight_norm(
            CausalConv1dSame(channels, channels, kernel_size, dilation=dilation)
        )
        self.conv2 = nn.utils.parametrizations.weight_norm(
            CausalConv1dSame(channels, channels, kernel_size, dilation=dilation)
        )

        # Use GELU for better performance
        self.act = nn.GELU()
        self.drop = nn.Dropout(p_drop)

        # Use GroupNorm for better stability with varying batch sizes
        self.norm1 = nn.GroupNorm(num_groups=min(8, channels), num_channels=channels)
        self.norm2 = nn.GroupNorm(num_groups=min(8, channels), num_channels=channels)

        # Initialize weights properly
        self._init_weights()

    def _init_weights(self):
        """Initialize weights using modern best practices."""
        # Initialize before applying weight norm
        pass  # Weight norm will handle initialization

    def forward(self, x: torch.Tensor) -> torch.Tensor:  # x: (B, C, T)
        residual = x

        # First convolution path
        out = self.conv1(x)  # (B, C, T)
        out = self.norm1(out)
        out = self.act(out)
        out = self.drop(out)

        # Second convolution path
        out = self.conv2(out)
        out = self.norm2(out)
        out = self.act(out)
        out = self.drop(out)

        return out + residual  # residual connection


class TCN(nn.Module):
    """
    Many-to-one TCN:
      input: (B, F_in, W)
      output: (B, 7)  for next-step efforts
    """

    def __init__(
        self,
        in_dim: int,
        out_dim: int = 7,
        channels: int = 64,
        kernel_size: int = 3,
        dilations: Tuple[int, ...] = (1, 2, 4, 8, 16),
        p_drop: float = 0.1,
    ):
        super().__init__()

        # Input projection with proper initialization
        self.inp = nn.Conv1d(in_dim, channels, kernel_size=1)
        nn.init.kaiming_normal_(self.inp.weight, mode="fan_out", nonlinearity="relu")

        # TCN blocks
        self.blocks = nn.ModuleList([TCNBlock(channels, kernel_size, dilation=d, p_drop=p_drop) for d in dilations])

        # Modern head with residual connection and better regularization
        self.head = nn.Sequential(
            nn.Conv1d(channels, channels, kernel_size=1),
            nn.GroupNorm(num_groups=min(8, channels), num_channels=channels),
            nn.GELU(),
            nn.Dropout(p_drop),
            nn.Conv1d(channels, out_dim, kernel_size=1),
        )

        # Initialize head weights
        for layer in self.head:
            if isinstance(layer, nn.Conv1d):
                nn.init.kaiming_normal_(layer.weight, mode="fan_out", nonlinearity="relu")
                if layer.bias is not None:
                    nn.init.constant_(layer.bias, 0)

    def forward(self, x: torch.Tensor) -> torch.Tensor:  # x: (B, F, W)
        h = self.inp(x)  # (B, C, W)

        # Apply TCN blocks sequentially
        for block in self.blocks:
            h = block(h)  # (B, C, W)

        y_seq = self.head(h)  # (B, out_dim, W)
        y = y_seq[:, :, -1]  # take last time step → (B, out_dim)
        return y


# ----------------------------
# 4) Modern Train / Eval
# ----------------------------
def train_one_epoch(
    model: nn.Module,
    loader: DataLoader,
    optimizer: torch.optim.Optimizer,
    loss_fn: nn.Module,
    grad_clip_norm: float = 1.0,
) -> float:
    """Optimized training loop with gradient clipping."""
    model.train()
    total_loss, num_samples = 0.0, 0
    for X, y in loader:
        X: torch.Tensor = X.to(DEVICE, non_blocking=True)
        y: torch.Tensor = y.to(DEVICE, non_blocking=True)

        optimizer.zero_grad(set_to_none=True)  # More efficient than zero_grad()

        pred = model(X)
        loss: torch.Tensor = loss_fn(pred, y)
        loss.backward()
        torch.nn.utils.clip_grad_norm_(model.parameters(), grad_clip_norm)
        optimizer.step()

        total_loss += loss.item() * X.size(0)
        num_samples += X.size(0)

    return total_loss / num_samples


@torch.no_grad()
def evaluate(model: nn.Module, loader: DataLoader, loss_fn: nn.Module) -> float:
    """Optimized evaluation with proper inference mode."""
    model.eval()
    total_loss, num_samples = 0.0, 0

    with torch.inference_mode():  # More efficient than torch.no_grad()
        for X, y in loader:
            X: torch.Tensor = X.to(DEVICE, non_blocking=True)
            y: torch.Tensor = y.to(DEVICE, non_blocking=True)

            pred = model(X)
            loss: torch.Tensor = loss_fn(pred, y)

            total_loss += loss.item() * X.size(0)
            num_samples += X.size(0)

    return total_loss / num_samples


# ----------------------------
# 5) End-to-end example
# ----------------------------
def make_dataloaders(
    df: pd.DataFrame, window: int, batch_size: int
) -> Tuple[DataLoader, DataLoader, DataLoader]:
    """Create optimized data loaders with modern best practices."""
    # Ensure column order & select features
    df = df[ALL_COLUMNS].copy()

    # Train/val/test split by time (no shuffling)
    N = len(df)
    train_end = int(N * 0.7)
    val_end = int(N * 0.85)
    df_train = df.iloc[:train_end]
    df_val = df.iloc[train_end:val_end]
    df_test = df.iloc[val_end:]

    X_train = df_train.values
    X_val = df_val.values
    X_test = df_test.values

    # Target indices for effort columns
    target_idx = [ALL_COLUMNS.index(c) for c in TARGET_COLUMNS]

    # Create datasets
    ds_train = WindowedNextStepDataset(X_train, target_idx, window)
    ds_val = WindowedNextStepDataset(X_val, target_idx, window)
    ds_test = WindowedNextStepDataset(X_test, target_idx, window)

    # Optimized data loaders
    num_workers = min(4, torch.get_num_threads())  # Adaptive worker count
    pin_memory = torch.cuda.is_available()  # Pin memory for GPU acceleration

    dl_train = DataLoader(
        ds_train,
        batch_size=batch_size,
        shuffle=True,
        num_workers=num_workers,
        collate_fn=collate_to_channels_last,
        drop_last=True,
        pin_memory=pin_memory,
        persistent_workers=num_workers > 0,  # Keep workers alive
        prefetch_factor=2 if num_workers > 0 else None,
    )

    dl_val = DataLoader(
        ds_val,
        batch_size=batch_size,
        shuffle=False,
        num_workers=num_workers,
        collate_fn=collate_to_channels_last,
        pin_memory=pin_memory,
        persistent_workers=num_workers > 0,
        prefetch_factor=2 if num_workers > 0 else None,
    )

    dl_test = DataLoader(
        ds_test,
        batch_size=batch_size,
        shuffle=False,
        num_workers=num_workers,
        collate_fn=collate_to_channels_last,
        pin_memory=pin_memory,
        persistent_workers=num_workers > 0,
        prefetch_factor=2 if num_workers > 0 else None,
    )

    return dl_train, dl_val, dl_test


def run_training(
    df: pd.DataFrame,
) -> Tuple[nn.Module, StandardScaler, Optional[StandardScaler], Tuple[DataLoader, DataLoader, DataLoader]]:
    """Modern training pipeline with best practices."""
    print(f"Training on device: {DEVICE}")

    # Create data loaders
    dl_train, dl_val, dl_test = make_dataloaders(df, window=WINDOW, batch_size=BATCH_SIZE)

    # Initialize model
    model = TCN(
        in_dim=len(ALL_COLUMNS),
        out_dim=len(TARGET_COLUMNS),
        channels=CHANNELS,
        kernel_size=KERNEL,
        dilations=DILATIONS,
        p_drop=DROPOUT,
    ).to(DEVICE)

    # Modern optimizer with weight decay
    optimizer = torch.optim.AdamW(
        model.parameters(),
        lr=LR,
        weight_decay=WEIGHT_DECAY,
        eps=1e-8,  # Improved numerical stability
    )

    # Learning rate scheduler
    scheduler = torch.optim.lr_scheduler.CosineAnnealingLR(optimizer, T_max=EPOCHS, eta_min=LR * 0.01)

    # Loss function - Huber loss is robust to outliers
    loss_fn = nn.SmoothL1Loss()

    # Training state tracking
    best_val_loss = math.inf
    best_state = None
    patience_counter = 0

    print(f"Model parameters: {sum(p.numel() for p in model.parameters() if p.requires_grad):,}")
    print(f"Receptive field: {receptive_field(KERNEL, DILATIONS)}")

    # Training loop
    for epoch in range(1, EPOCHS + 1):
        # Training phase
        train_loss = train_one_epoch(model, dl_train, optimizer, loss_fn, GRAD_CLIP_NORM)

        # Validation phase
        val_loss = evaluate(model, dl_val, loss_fn)

        # Learning rate scheduling
        scheduler.step()
        current_lr = scheduler.get_last_lr()[0]

        print(f"Epoch {epoch:03d} | train {train_loss:.4f} | val {val_loss:.4f} | lr {current_lr:.2e}")

        # Model checkpointing with early stopping
        if val_loss < best_val_loss:
            best_val_loss = val_loss
            best_state = {
                k: (v.cpu().clone() if isinstance(v, torch.Tensor) else v) for k, v in model.state_dict().items()
            }
            patience_counter = 0
        else:
            patience_counter += 1

        # Early stopping
        if patience_counter >= PATIENCE:
            print(f"Early stopping at epoch {epoch} (patience: {PATIENCE})")
            break

    # Load best model
    if best_state is not None:
        model.load_state_dict(best_state)
        print(f"Loaded best model with validation loss: {best_val_loss:.4f}")

    # Final evaluation
    test_loss = evaluate(model, dl_test, loss_fn)
    print(f"Test loss: {test_loss:.4f}")

    return model (dl_train, dl_val, dl_test)


# ----------------------------
# 6) Optimized Inference
# ----------------------------
@torch.inference_mode()
def predict_next_efforts(model: nn.Module, feat_scaler: StandardScaler, recent_window_df: pd.DataFrame) -> np.ndarray:
    """
    Optimized inference with proper device handling and memory management.

    Args:
        model: Trained TCN model
        feat_scaler: Fitted feature scaler from training
        recent_window_df: Last WINDOW rows with ALL_COLUMNS (unscaled)

    Returns:
        (7,) predicted next-step efforts in scaled space
    """
    assert list(recent_window_df.columns) == ALL_COLUMNS, f"Column order mismatch. Expected {ALL_COLUMNS}"
    assert len(recent_window_df) == WINDOW, f"Expected {WINDOW} rows, got {len(recent_window_df)}"

    model.eval()

    # Preprocess input
    x = feat_scaler.transform(recent_window_df.values)  # (W, F)
    x = torch.from_numpy(x).float().unsqueeze(0)  # (1, W, F)
    x = x.transpose(1, 2).to(DEVICE, non_blocking=True)  # (1, F, W)

    # Standard inference
    y: torch.Tensor = model(x)[0]  # (7,)

    return y.cpu().numpy()


# ----------------------------
# 7) Receptive field sanity check (optional)
# ----------------------------
def receptive_field(kernel: int, dilations: Sequence[int], convs_per_block: int = 2) -> int:
    """
    R = 1 + (k-1) * sum(convs_per_block * d for d in dilations)
    """
    return 1 + (kernel - 1) * sum(convs_per_block * d for d in dilations)


def count_parameters(model: nn.Module) -> int:
    """Count the number of trainable parameters in a model."""
    return sum(p.numel() for p in model.parameters() if p.requires_grad)


def save_model(model: nn.Module, scaler: StandardScaler, filepath: str):
    """Save model and scaler for later use."""
    torch.save(
        {
            "model_state_dict": model.state_dict(),
            "scaler": scaler,
            "model_config": {
                "in_dim": len(ALL_COLUMNS),
                "out_dim": len(TARGET_COLUMNS),
                "channels": CHANNELS,
                "kernel_size": KERNEL,
                "dilations": DILATIONS,
                "p_drop": DROPOUT,
            },
        },
        filepath,
    )


def load_model(filepath: str, device: Optional[torch.device] = None) -> Tuple[nn.Module, StandardScaler]:
    """Load saved model and scaler."""
    if device is None:
        device = get_device()

    checkpoint = torch.load(filepath, map_location=device)

    # Recreate model
    model = TCN(**checkpoint["model_config"]).to(device)
    model.load_state_dict(checkpoint["model_state_dict"])

    return model, checkpoint["scaler"]


if __name__ == "__main__":
    # Set random seeds for reproducibility
    torch.manual_seed(42)
    np.random.seed(42)

    # Enable optimizations
    torch.backends.cudnn.benchmark = True  # Optimize for fixed input sizes
    torch.backends.cudnn.deterministic = False  # Allow non-deterministic for speed

    print("PyTorch Optimized TCN for Robot Joint Effort Prediction")
    print(f"Device: {DEVICE}")

    # Example usage:
    df = pd.read_csv("task0_data.csv")
    print(f"Loaded data with shape: {df.shape}")

    model, scaler, _, _ = run_training(df)

    # Save trained model
    save_model(model, scaler, "tcn_model.pth")
    print("Model saved to tcn_model.pth")

    # Print model info
    print(f"Model parameters: {count_parameters(model):,}")
    print(f"Receptive field: {receptive_field(KERNEL, DILATIONS)}")
