# Demo policy artifact

`policy.pt` is the TorchScript artifact consumed by `brov_control/policy_node`.
`metadata.yaml` records its observation/action contract and the expected vehicle model hash;
`sha256.txt` permits an independent integrity check.

This file is only 33 KB, so it is stored directly in Git rather than Git LFS. Replacing it
requires updating both metadata/checksum files and rerunning `make check`.

The training/export implementation remains outside this real-robot runtime repository.
