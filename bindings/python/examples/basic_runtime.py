import argparse

import open_lmm


def run(config_directory: str) -> open_lmm.RuntimeSnapshot:
    with open_lmm.Runtime() as runtime:
        runtime.open(config_directory, label="python-example")
        runtime.run_all().wait()
        return runtime.snapshot()


def main() -> None:
    parser = argparse.ArgumentParser(description="Run the OpenLMM Python example")
    parser.add_argument("config_directory")
    args = parser.parse_args()
    snapshot = run(args.config_directory)
    print(
        f"status={snapshot.status.name} "
        f"runtime_revision={snapshot.pipeline.runtime_revision} "
        f"config_revision={snapshot.pipeline.config_revision} "
        f"output_directory={snapshot.output_directory}"
    )


if __name__ == "__main__":
    main()
