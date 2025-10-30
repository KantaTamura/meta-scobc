# meta-scobc

## Prerequisite

> [!IMPORTANT]
> We store `.xsa` files using [Git LFS][1]. Make sure Git LFS is
> installed; otherwise you won't get the actual `.xsa` files and the build
> will fail.

## Setup

```
$ uv sync
$ source .venv/bin/activate
```

## Build

|Machine|Build command|
|-|-|
|Kria K24 SOM board|`$ kas build kas/xilinx-k24-smk-sdt.yml`|
|SC-OBC Module V1|`$ kas build kas/scobc-v1.yml`|

### Faster builds (shared caches)

```
$ kas build kas/xilinx-k24-smk-sdt.yml:kas/conf/site.opt.yml
```

[1]: https://git-lfs.com/
