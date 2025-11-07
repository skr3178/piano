# RoboPianist: Dexterous Piano Playing with Deep Reinforcement Learning

[![build][tests-badge]][tests]
[![docs][docs-badge]][docs]
[![PyPI Python Version][pypi-versions-badge]][pypi]
[![PyPI version][pypi-badge]][pypi]

[tests-badge]: https://github.com/google-research/robopianist/actions/workflows/ci.yml/badge.svg
[docs-badge]: https://github.com/google-research/robopianist/actions/workflows/docs.yml/badge.svg
[tests]: https://github.com/google-research/robopianist/actions/workflows/ci.yml
[docs]: https://google-research.github.io/robopianist/
[pypi-versions-badge]: https://img.shields.io/pypi/pyversions/robopianist
[pypi-badge]: https://badge.fury.io/py/robopianist.svg
[pypi]: https://pypi.org/project/robopianist/

[![Video](http://img.youtube.com/vi/VBFn_Gg0yD8/hqdefault.jpg)](https://youtu.be/VBFn_Gg0yD8)

RoboPianist is a new benchmarking suite for high-dimensional control, targeted at testing high spatial and temporal precision, coordination, and planning, all with an underactuated system frequently making-and-breaking contacts. The proposed challenge is *mastering the piano* through bi-manual dexterity, using a pair of simulated anthropomorphic robot hands.

This codebase contains software and tasks for the benchmark, and is powered by [MuJoCo](https://mujoco.org/).

- [Latest Updates](#latest-updates)
- [Getting Started](#getting-started)
- [Installation](#installation)
  - [Install from source](#install-from-source)
  - [Install from PyPI](#install-from-pypi)
  - [Optional: Download additional soundfonts](#optional-download-additional-soundfonts)
- [MIDI Dataset](#midi-dataset)
- [CLI](#cli)
- [Contributing](#contributing)
- [FAQ](#faq)
- [Citing RoboPianist](#citing-robopianist)
- [Acknowledgements](#acknowledgements)
- [Works that have used RoboPianist](#works-that-have-used-robopianist)
- [License and Disclaimer](#license-and-disclaimer)

-------

## Latest Updates

- [24/12/2023] Updated install script so that it checks out the correct Menagerie commit. Please re-run `bash scripts/install_deps.sh` to update your installation.
- [17/08/2023] Added a [pixel wrapper](robopianist/wrappers/pixels.py) for augmenting the observation space with RGB images.
- [11/08/2023] Code to train the model-free RL policies is now public, see [robopianist-rl](https://github.com/kevinzakka/robopianist-rl).

-------

## Getting Started

We've created an introductory [Colab](https://colab.research.google.com/github/google-research/robopianist/blob/main/tutorial.ipynb) notebook that demonstrates how to use RoboPianist. It includes code for loading and customizing a piano playing task, and a demonstration of a pretrained policy playing a short snippet of *Twinkle Twinkle Little Star*. Click the button below to get started!

[![Open In Colab](https://colab.research.google.com/assets/colab-badge.svg)](https://colab.research.google.com/github/google-research/robopianist/blob/main/tutorial.ipynb)

## Installation

RoboPianist is supported on both Linux and macOS and can be installed with Python >= 3.8. We recommend using [Miniconda](https://docs.conda.io/en/latest/miniconda.html) to manage your Python environment.

### Install from source

The recommended way to install this package is from source. Start by cloning the repository:

```bash
git clone https://github.com/google-research/robopianist.git && cd robopianist
```

Next, install the prerequisite dependencies:

```bash
git submodule init && git submodule update
bash scripts/install_deps.sh
```

Finally, create a new conda environment and install RoboPianist in editable mode:

```bash
conda create -n pianist python=3.10
conda activate pianist

pip install -e ".[dev]"
```

To test your installation, run `make test` and verify that all tests pass.

### Install from PyPI

First, install the prerequisite dependencies:

```bash
bash <(curl -s https://raw.githubusercontent.com/google-research/robopianist/main/scripts/install_deps.sh) --no-soundfonts
```

Next, create a new conda environment and install RoboPianist:

```bash
conda create -n pianist python=3.10
conda activate pianist

pip install --upgrade robopianist
```

### Optional: Download additional soundfonts

We recommend installing additional soundfonts to improve the quality of the synthesized audio. You can easily do this using the RoboPianist CLI:

```bash
robopianist soundfont --download
```

For more soundfont-related commands, see [docs/soundfonts.md](docs/soundfonts.md).

## MIDI Dataset

The PIG dataset cannot be redistributed on GitHub due to licensing restrictions. See [docs/dataset](docs/dataset.md) for instructions on where to download it and how to preprocess it.

## CLI

RoboPianist comes with a command line interface (CLI) that can be used to download additional soundfonts, play MIDI files, preprocess the PIG dataset, and more. For more information, see [docs/cli.md](docs/cli.md).

## Contributing

We welcome contributions to RoboPianist. Please see [docs/contributing.md](docs/contributing.md) for more information.

## FAQ

See [docs/faq.md](docs/faq.md) for a list of frequently asked questions.

## Citing RoboPianist

If you use RoboPianist in your work, please use the following citation:

```bibtex
@inproceedings{robopianist2023,
  author = {Zakka, Kevin and Wu, Philipp and Smith, Laura and Gileadi, Nimrod and Howell, Taylor and Peng, Xue Bin and Singh, Sumeet and Tassa, Yuval and Florence, Pete and Zeng, Andy and Abbeel, Pieter},
  title = {RoboPianist: Dexterous Piano Playing with Deep Reinforcement Learning},
  booktitle = {Conference on Robot Learning (CoRL)},
  year = {2023},
}
```

## Acknowledgements

We would like to thank the following people for making this project possible:

- [Philipp Wu](https://www.linkedin.com/in/wuphilipp/) and [Mohit Shridhar](https://mohitshridhar.com/) for being a constant source of inspiration and support.
- [Ilya Kostrikov](https://www.kostrikov.xyz/) for constantly raising the bar for RL engineering and for invaluable debugging help.
- The [Magenta](https://magenta.tensorflow.org/) team for helpful pointers and feedback.
- The [MuJoCo](https://mujoco.org/) team for the development of the MuJoCo physics engine and their support throughout the project.

## Works that have used RoboPianist

- *Privileged Sensing Scaffolds Reinforcement Learning*, Hu et. al. ([paper](https://openreview.net/forum?id=EpVe8jAjdx), [website](https://penn-pal-lab.github.io/scaffolder/))

## License and Disclaimer

[MuJoco Menagerie](https://github.com/deepmind/mujoco_menagerie)'s license can be found [here](https://github.com/deepmind/mujoco_menagerie/blob/main/LICENSE). Soundfont licensing information can be found [here](docs/soundfonts.md). MIDI licensing information can be found [here](docs/dataset.md). All other code is licensed under an [Apache-2.0 License](LICENSE).

This is not an officially supported Google product.


## For running and training this dataset the following resources were used:

Algorithm and Framework

RL Algorithm: DroQ — Dropout Q-functions for doubly efficient reinforcement learning
→ a regularized variant of Soft Actor-Critic (SAC)

Implementation framework: JAX (from Google)

ROBOPIANIST
Physics simulation: MuJoCo (versions cited from Todorov et al. 2012 and dm_control 2020)
Environment source: MuJoCo Menagerie (for Shadow Dexterous Hand models)
Observation frequency: 20 Hz control, 500 Hz physics update
Observation space: proprioception + future goal states (lookahead horizon 
𝐿
L)
Action space: 45D (joint angles + sustain pedal)
Reward terms:

Key press accuracy
Finger proximity to target keys
Energy minimization penalty

⚙️ Training Infrastructure
Hardware: Google Cloud n1-highmem-64
Intel Xeon E5-2696 v3 CPU (32 cores @ 2.3 GHz)
416 GB RAM
4 × Tesla K80 GPUs
Parallelization: up to 8 simultaneous runs
Typical run time: ≈ 5 hours per song (5 million steps per run)
Optimizer: Adam (lr = 3 × 10⁻⁴, β₁ = 0.9, β₂ = 0.999)

Network:

Actor & Critic = 3-layer MLPs (256 neurons, ReLU, dropout 0.01, layer norm)

Xavier weight initialization

Diagonal Gaussian actor (tanh-squashed)

🎹 Environment Details

Robot Hands: Two anthropomorphic Shadow Dexterous Hands, 44 DOF total

Instrument: Full 88-key digital piano modeled with linear-spring keys

Dataset: ROBOPIANIST-REPERTOIRE-150, based on annotated MIDI + fingering data from the PIG dataset

Evaluation metric: F1 score (precision × recall of correct key activations)

🧩 Baselines Used

Model-based baseline: MPC (Predictive Sampling) implemented in C++/MJPC

Evaluated on a MacBook Pro M1 Max (64 GB RAM)

0.2 s planning horizon, 0.01 s step, 0.005 s physics step

✅ In summary:
ROBOPIANIST was trained in MuJoCo using JAX-based DroQ (a SAC variant) on a Google Cloud high-memory 64-core CPU + 4 K80 GPU machine.
Simulation and environment were built from MuJoCo Menagerie’s Shadow Hand and a custom full-piano model, with training guided by human fingering priors and MIDI-based reward shaping.
