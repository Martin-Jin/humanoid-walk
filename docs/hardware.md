# Hardware use: what runs on the CPU and the GPU

TODO: fill in once the network and training loop exist.

## Choosing the device

`utils/device.py` resolves the training device:

- `"auto"` uses CUDA when PyTorch can see a GPU, otherwise the CPU.
- `"cpu"` or `"cuda"` forces a device. `"cuda"` raises an error if CUDA is unavailable.

## Where each step runs

| Step                              | Device          | Why                                          |
| --------------------------------- | --------------- | -------------------------------------------- |
| Physics simulation, rewards       | CPU             | TODO. Must not depend on a GPU or on pygame. |
| Action selection during rollouts  | TODO            | TODO                                         |
| PPO update                        | Training device | TODO                                         |
