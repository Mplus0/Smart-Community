"""Only launch groups recorded by this Infra installation may be stopped."""
import json
import shlex
import subprocess
import uuid
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


class Processes:
    def __init__(self, config):
        self.config = config
        self.root = ROOT / 'data'
        self.root.mkdir(exist_ok=True)
        self.file = self.root / 'processes.json'
        self.records = json.loads(self.file.read_text()) if self.file.exists() else {}

    def run(self, script):
        host = self.config['ssh_host']
        command = ['ssh', '-o', 'BatchMode=yes', '-o', 'ConnectTimeout=3', host,
                   'bash -lc ' + shlex.quote(script)] if host else ['bash', '-lc', script]
        result = subprocess.run(command, capture_output=True, text=True, timeout=12)
        if result.returncode:
            raise ValueError(result.stderr.strip() or result.stdout.strip())
        return result.stdout.strip()

    def environment(self):
        c = self.config
        ip = c['master_uri'].split('//')[1].split(':')[0] if c['ssh_host'] else c['ros_ip']
        return ('source /opt/ros/noetic/setup.bash && source ' +
                shlex.quote(c['remote_workspace'] + '/devel/setup.bash') +
                ' && unset ROS_HOSTNAME && export ROS_MASTER_URI=' + shlex.quote(c['master_uri']) +
                ' ROS_IP=' + shlex.quote(ip) + ' && ')

    def status(self):
        if not self.records:
            return {}
        script = '\n'.join('kill -0 -- -' + str(r['pid']) + ' 2>/dev/null && echo ' + name + ' || true'
                           for name, r in self.records.items())
        alive = self.run(script).splitlines()
        return {k: dict(v, running=k in alive) for k, v in self.records.items()}

    def start(self, name, args):
        if self.status().get(name, {}).get('running'):
            raise ValueError('该服务已经由 Infra 启动')
        token = uuid.uuid4().hex
        log = '/tmp/community-infra-' + token + '.log'
        command = self.environment() + 'exec ' + shlex.join(args)
        pid = int(self.run('nohup setsid bash -lc ' + shlex.quote(command) +
                           ' >' + shlex.quote(log) + ' 2>&1 < /dev/null & echo $!'))
        self.records[name] = dict(pid=pid, token=token, log=log)
        self.file.write_text(json.dumps(self.records))
        return self.records[name]

    def stop(self, name):
        record = self.records.get(name)
        if not record:
            raise ValueError('只能停止 Infra 自己启动的进程')
        # Verify the unique log is still open before signaling a persisted PID group.
        pid = str(record['pid'])
        self.run('if kill -0 -- -' + pid + ' 2>/dev/null; then '
                 'test "$(readlink /proc/' + pid + '/fd/1)" = ' + shlex.quote(record['log']) +
                 ' && kill -INT -- -' + pid + '; fi')

    def results(self):
        return json.loads(self.run('python3 -c ' + shlex.quote(
            "import pathlib,json; p=sorted(pathlib.Path(" + repr(self.config['remote_results']) +
            ").glob('*/mission_results.json'),key=lambda p:p.stat().st_mtime); "
            "print(json.dumps({'result':json.loads(p[-1].read_text()),'log':"
            "(p[-1].parent/'mission.log').read_text()[-20000:]} if p else {}))")))
