"""Host test of production MA732 sources; no target or hardware access."""
from pathlib import Path
import subprocess
import tempfile

root = Path(__file__).resolve().parents[2]
test_dir = Path(__file__).resolve().parent
with tempfile.TemporaryDirectory(prefix="ma732-register-test-") as build_dir:
    binary = Path(build_dir) / "register-test"
    subprocess.run([
        "g++", "-std=c++20", "-Wall", "-Wextra", "-Werror",
        "-I" + str(test_dir / "mocks"),
        "-I" + str(root / "Application/Sensor/Inc"),
        str(test_dir / "register_test.cpp"),
        str(root / "Application/Sensor/Src/MA732.cpp"),
        str(root / "Application/Sensor/Src/MA732_debug.cpp"),
        "-o", str(binary),
    ], check=True)
    subprocess.run([str(binary)], check=True)
