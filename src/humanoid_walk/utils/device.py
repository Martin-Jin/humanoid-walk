"""Choose the PyTorch device. PyTorch is imported lazily so headless code works without it."""


def resolve_device(preference: str = "auto") -> str:
    """Return ``"cpu"`` or ``"cuda"`` from ``"auto"``, ``"cpu"`` or ``"cuda"``."""
    if preference not in ("auto", "cpu", "cuda"):
        raise ValueError(f"Unknown device preference: {preference!r}")
    if preference == "cpu":
        return "cpu"
    import torch

    available = torch.cuda.is_available()
    if preference == "cuda" and not available:
        raise RuntimeError("CUDA was requested but PyTorch cannot see a GPU.")
    return "cuda" if available else "cpu"


def configure_torch(device: str, num_threads: int = 1) -> None:
    """Apply process-wide PyTorch settings for training on ``device``."""
    import torch

    torch.set_num_threads(num_threads)
    if device == "cuda":
        torch.set_float32_matmul_precision("high")
