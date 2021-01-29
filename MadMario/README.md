
# MadMario
PyTorch official tutorial to build an AI-powered Mario.

## Set Up
1. Install [conda](https://www.anaconda.com/products/individual)
2. Install dependencies with `environment.yml`
    ```
    conda env create -f environment.yml
    ```
    Check the new environment *mario* is [created successfully](https://docs.conda.io/projects/conda/en/latest/user-guide/tasks/manage-environments.html#creating-an-environment-from-an-environment-yml-file).
    ```
    conda env list
    ```

3. Activate *mario* enviroment
    ```
    conda activate mario
    ```

4. Install dependent python packages.
    ```
    pip install -r requirements.txt  -i https://pypi.tuna.tsinghua.edu.cn/simple
    ```
    **Note:** In case of "Read timed out" error, add `--default-timeout=100` in the above command. You may need to try a couple of time due to poor internet connection. In extreme cases, copy the link (e.g. [opencv-python](https://pypi.tuna.tsinghua.edu.cn/packages/0b/61/843ab00a3ed67f3f50be786bd9c78ff52c55841a13f26f8cb3cd8502eb09/opencv_python-4.2.0.34-cp38-cp38-manylinux1_x86_64.whl)) and download the installation file in your browser and install it locally by running `pip install opencv_python-4.2.0.34-cp38-cp38-manylinux1_x86_64.whl`.

## Running
To start the **learning** process for Mario, run
```
python main.py
```
This starts the *double Q-learning* and logs key training metrics to `checkpoints`. In addition, a copy of `MarioNet` and current exploration rate will be saved.

GPU will automatically be used if available. Training time is around 80 hours on CPU and 20 hours on GPU.

To **evaluate** a trained Mario,
```
python replay.py
```
This visualizes Mario playing the game in a window. Performance metrics will be logged to a new folder under `checkpoints`. Change the `load_dir`, e.g. `checkpoints/2020-06-06T22-00-00`, in `Mario.load()` to check a specific timestamp.


## Project Structure
**main.py**
Main loop between Environment and Mario

**agent.py**
Define how the agent collects experiences, makes actions given observations and updates the action policy.

**wrappers.py**
Environment pre-processing logics, including observation resizing, rgb to grayscale, etc.

**neural.py**
Define Q-value estimators backed by a convolution neural network.

**metrics.py**
Define a `MetricLogger` that helps track training/evaluation performance.

**tutorial.ipynb**
Interactive tutorial with extensive explanation and feedback. Run it on [Google Colab](https://colab.research.google.com/notebooks/intro.ipynb#recent=true).

## Key Metrics

- Episode: current episode
- Step: total number of steps Mario played
- Epsilon: current exploration rate
- MeanReward: moving average of episode reward in past 100 episodes
- MeanLength: moving average of episode length in past 100 episodes
- MeanLoss: moving average of step loss in past 100 episodes
- MeanQValue: moving average of step Q value (predicted) in past 100 episodes

## Pre-trained

Checkpoint for a trained Mario: https://drive.google.com/file/d/1RRwhSMUrpBBRyAsfHLPGt1rlYFoiuus2/view?usp=sharing

