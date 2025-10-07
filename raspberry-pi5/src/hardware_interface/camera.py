import subprocess
from pathlib import Path


# Execute a simple command
def run():
    here = Path(__file__).resolve().parent
    mcm = (here / ".." / "resources" / "mavlink-camera-manager").resolve()
    mcm.chmod(mcm.stat().st_mode | 0o111)

    result = subprocess.Popen(
        [str(mcm),
         "--mavlink=udpout:0.0.0.0:14000",
         "--verbose", "--signalling-server", "ws://0.0.0.0:6021"], stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)

# ~/mavlink-camera-manager/target/release/mavlink-camera-manager --mavlink=udpout:0.0.0.0:14000 --verbose --signalling-server ws://0.0.0.0:6021