# meta-scobc

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
