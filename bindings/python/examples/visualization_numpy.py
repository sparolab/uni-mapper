import numpy as np
import open_lmm


def xyz(config_directory: str, agent: str) -> np.ndarray:
    with open_lmm.Runtime() as runtime:
        runtime.open(config_directory, label="visualization-example")
        runtime.run_stage(open_lmm.Stage.DATA_LOAD).wait()
        return runtime.visualization(agent).points[:, :3]
