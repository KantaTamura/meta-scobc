## Setup

```
$ uv sync
$ source .venv/bin/activate
```

## Build

```
$ kas build kas/xilinx-k24-smk-sdt.yml
```

### Faster builds (shared caches)

```
$ kas build kas/xilinx-k24-smk-sdt.yml:kas/conf/site.opt.yml
```
