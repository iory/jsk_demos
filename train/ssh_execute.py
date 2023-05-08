import configparser
from pathlib import Path
import datetime
import paramiko
from paramiko.config import SSH_PORT
import scp

from eos import makedirs


def ssh_out(stderr, stdout, host, username, cmd):
    s = ""
    for e in stderr:
        s = s + e
    if 0 < len(s):
        print(s)

    s = ""
    for o in stdout:
        s = s + o
    if 0 < len(s):
        print(s)



def ssh_cmd(ssh_config, host, key_file, passphrase, list_cmd):
    lookup = ssh_config.lookup(host)

    hostname = lookup["hostname"]
    username = lookup["user"]

    try:
        port = int(lookup["port"])
    except Exception:
        port = SSH_PORT

    key_filename = lookup['identityfile']
    try:
        pkey = paramiko.RSAKey.from_private_key_file(key_file, passphrase)
    except Exception:
        try:
            pkey = paramiko.ECDSAKey.from_private_key_file(key_file, passphrase)
        except Exception:
            try:
                pkey = paramiko.Ed25519Key.from_private_key_file(key_file, passphrase)
            except Exception:
                pkey = None

    try:
        sock = paramiko.ProxyJump(lookup["proxyjump"])
    except Exception:
        try:
            print(lookup["proxycommand"])
            sock = paramiko.ProxyCommand(lookup["proxycommand"])
        except Exception:
            sock = None
    ssh_client = paramiko.SSHClient()
    try:
        ssh_client.load_system_host_keys()
        ssh_client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
        print(hostname)
        print(port)
        print(key_filename)
        print(sock)
        ssh_client.connect(
            hostname=hostname,
            port=port,
            username=username,
            key_filename=key_filename,
            pkey=pkey,
            sock=sock,
            )
        for cmd in list_cmd:
            if len(cmd) == 0:
                continue
            if cmd.split()[0] == 'scp':
                tmps = cmd.split()
                if len(tmps) < 3:
                    continue
                print('Executing scp: {}'.format(cmd))
                with scp.SCPClient(ssh_client.get_transport()) as tmp_scp:
                    makedirs(Path(tmps[2]).parent)
                    tmp_scp.get(remote_path=tmps[1],
                                local_path=tmps[2])
            else:
                print('Executing command: {}'.format(cmd))
                stdin, stdout, stderr = ssh_client.exec_command(cmd)
                ssh_out(stderr, stdout, host, username, cmd)
    finally:
        ssh_client.close()


def remote_train():
    ssh_config_file = '/home/iory/.ssh/config'
    vps_key_file = '/home/iory/.ssh/id_rsa'
    with open('/home/iory/.ssh/.pass') as f:
        vps_passphrase = f.read()

    ssh_config = paramiko.SSHConfig()
    ssh_config.parse(open(ssh_config_file, 'r'))

    host = "tmp"
    list_cmd = [
        'hostname',
        # 'cd /home/iory/junk/2023/01/thk && git show --format="%H" --no-patch',
        # "tmux new-session -d 'cd /home/iory/junk/2023/01/thk && python -- generate_data.py -n 500 -b 32 --target yamanashi'",
    ]
    ssh_cmd(ssh_config, host, vps_key_file, vps_passphrase, list_cmd)
    # subprocess.call('rsync --verbose {}:/tmp/yamanashi/yolov7-seg-coco/weights/best.pt /home/iory/thk/best.pt'.format(host),
    #                 shell=True)
