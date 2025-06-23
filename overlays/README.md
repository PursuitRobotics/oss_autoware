# Build Environment for Pursuit Robotics Autoware Stack


```bash

python3 -m venv .penv

source .penv/bin/actvate

```

Upgrade the Pip

```bash
pip3 install --upgrade pip
```


## Apply the requiremnts in the environ

```bash
export ENVIRON=staging

pip install -r ./environs/${ENVIRON}/requirements.txt

```

## Build it

```base

cd docker

./build.sh --no-cuda --platform linux/arm64 --devel-only --target universe-devel

```