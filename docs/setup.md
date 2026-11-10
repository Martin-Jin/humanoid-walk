# Development setup

## Requirements

- Python 3.10 or newer
- The Windows `py` launcher, or an equivalent Python command on another platform

## Install

From the repository root, create and activate a virtual environment:

```powershell
py -m venv .venv
.\.venv\Scripts\Activate.ps1
```

For a CPU-only installation, install the project with its CPU extra:

```powershell
py -m pip install --upgrade pip
py -m pip install -e ".[cpu]"
```

The project install provides Gymnasium, NumPy, pygame-ce, pymunk and PyYAML. The
CPU extra also installs the standard PyPI PyTorch wheel.

### NVIDIA CUDA installation

Install the PyTorch wheel built for CUDA before installing the project. This stops
pip from selecting the CPU wheel:

```powershell
py -m pip install --upgrade pip
py -m pip install torch --index-url https://download.pytorch.org/whl/cu126
py -m pip install -e .
```

The `cu126` index is an example. Choose the one that matches your driver using the
[official PyTorch selector](https://pytorch.org/get-started/locally/). Do not install
the `cpu` extra in this environment, because it requests the CPU wheel.

Verify that PyTorch can see the GPU:

```powershell
py -c "import torch; print(torch.__version__); print(torch.cuda.is_available())"
```

## Verify the checkout

```powershell
py -m compileall src tests scripts
py -m unittest discover -s tests
```

Both must pass with no errors.
