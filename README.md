# meta-scobc

## Setup

We use [`uv`](https://github.com/astral-sh/uv) to manage the Python environment for `kas` and related tooling.

```bash
$ uv sync
$ source .venv/bin/activate
```

## Build

Builds are managed using [`kas`](https://github.com/siemens/kas).

### Build command

To build a machine:

```bash
$ kas build <kas configuration file>
```

Example:

```bash
$ kas build kas/scobc-v1-sdt.yml
```

The resulting images will be available under:

```bash
build/tmp/deploy/images/<machine>/
```

### Supported Machines

| Machine                          | Machine configuration file                       | kas configuration file                      |
| -------------------------------- | -------------------------------------------------| ------------------------------------------- |
| SC-OBC Module V1 Space Grade     | [`versal-scobc-v1-sdt-full`][m-space]            | [`kas/scobc-v1-sdt.yml`][kas-space]         |
| SC-OBC Module V1 Developer Grade | [`versal-scobc-v1-ve2302e-sdt-full`][m-dev-2302] | [`kas/scobc-v1-devgrade.yml`][kas-dev-2302] |
| SC-OBC Module V1 VE2002 Variant  | [`versal-scobc-v1-ve2002e-sdt-full`][m-dev-2002] | [`kas/scobc-v1-devgrade-ve2002.yml`][kas-dev-2002] |

> [!NOTE]
> SC-OBC Module V1 VE2002 Variant is not
> commercially available.

### Faster builds (shared caches)

For faster incremental builds, you should keep the downloaded files and
the shared state cache. By default, the download directory and the
shared state directory are placed under the build directory. This means
that when you remove the build directory, those artifacts are removed as
well.

Instead, set the `DL_DIR` and `SSTATE_DIR` environment variables to
fixed locations outside the build directory. This keeps the artifacts
available even if you delete and recreate the build directory.

```console
$ export DL_DIR=/opt/yocto/downloads
$ export SSTATE_DIR=/opt/yocto/sstate-cache
$ kas build kas/scobc-v1-sdt.yml
```

[m-space]: meta-scobc/conf/machine/versal-scobc-v1-ve2302i-sdt-full.conf
[m-dev-2302]: meta-scobc/conf/machine/versal-scobc-v1-ve2302e-sdt-full.conf
[m-dev-2002]: meta-scobc/conf/machine/versal-scobc-v1-ve2002e-sdt-full.conf
[kas-space]: kas/scobc-v1-sdt.yml
[kas-dev-2302]: kas/scobc-v1-devgrade.yml
[kas-dev-2002]: kas/scobc-v1-devgrade-ve2002.yml
