"""Run the production shared-encoder receiver against simulated registers."""
import os
from pathlib import Path
import shlex
import subprocess
import tempfile

root = Path(__file__).resolve().parents[2]
compiler = shlex.split(os.environ.get("CXX", "g++"))
with tempfile.TemporaryDirectory(prefix="ma732-slave-test-") as folder:
    exe = Path(folder) / ("receiver_test.exe" if os.name == "nt" else "receiver_test")
    subprocess.run(compiler + [
        "-std=c++20", "-Wall", "-Wextra", "-Werror",
        "-I" + str(root / "tests/ma732_slave/mocks"),
        "-I" + str(root / "Application/Sensor/Inc"),
        str(root / "tests/ma732_slave/receiver_test.cpp"), "-o", str(exe),
    ], check=True)
    subprocess.run([str(exe)], check=True)
