#!/usr/bin/env python3
"""Offline W08 boundary tests. Docker is a test double, ELF fixtures are compiled.

These tests do NOT build/load the real runtime, ROS or acados, and cache-path
reuse is NOT measured Docker/CMake/Cargo cache reuse. Optional image-script
checks: XGC2_IMAGES_SOURCE=/path/to/xgc2-images python3 scripts/test-onboard-build.py
"""
import importlib.util
import json
import os
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest

SCRIPTS = Path(__file__).resolve().parent
DRIVER = SCRIPTS / 'build-onboard-artifacts.sh'
IMAGES = os.environ.get('XGC2_IMAGES_SOURCE')
IMAGE_SCRIPT = Path(IMAGES) / 'onboard-baseline/build-local-image.sh' if IMAGES else None

FAKE_DOCKER = r'''#!/usr/bin/env python3
import json, os, pathlib, shutil, sys
args = sys.argv[1:]
with open(os.environ['FAKE_LOG'], 'a') as log:
    log.write(json.dumps(args) + '\n')
if args[:1] == ['--host']:
    args = args[2:]
if args[:2] == ['context', 'inspect']:
    print(os.environ.get('FAKE_ENDPOINT', 'unix:///tmp/test-docker.sock'))
elif args[:2] == ['image', 'inspect']:
    if os.environ.get('FAKE_MISSING_IMAGE'):
        print('fixture: no such local image', file=sys.stderr); sys.exit(9)
    fmt = args[args.index('--format') + 1]
    arch = os.environ.get('FAKE_ARCH', 'amd64')
    if '.Id' in fmt and '.Architecture' in fmt:
        print('sha256:' + os.environ.get('FAKE_IMAGE_ID', 'a'*64) + ' linux/' + arch)
    elif '.Architecture' in fmt:
        print('linux/' + arch)
    elif '.Entrypoint' in fmt:
        print('["/opt/xgc2/onboard-baseline/image-entrypoint.sh"]')
    else:
        print('fixture local image')
elif args[:1] == ['build']:
    sys.exit(int(os.environ.get('FAKE_BUILD_RC', '0')))
elif args[:1] == ['run']:
    mounts = {}
    for index, arg in enumerate(args):
        if arg == '--mount':
            fields = dict(part.split('=', 1) for part in args[index+1].split(',') if '=' in part)
            mounts[fields['dst']] = pathlib.Path(fields['src'])
    if os.environ.get('FAKE_BUILD_RC'):
        print('fixture: compiler/linker failed', file=sys.stderr)
        sys.exit(int(os.environ['FAKE_BUILD_RC']))
    if os.environ.get('FAKE_EMPTY_OUTPUT'):
        sys.exit(0)
    out = mounts['/workspace/output']
    fixture = pathlib.Path(os.environ['FAKE_ELF'])
    for directory, names in {
        'bin': ['xgc-rt-host', 'xgc-rt-render', 'xgc-rt-audit'],
        'plugins': ['libplan_dmpc.so', 'libros_io.so', 'libnumeric_vehicle.so', 'libdmpc_rounds.so', 'libstation_io.so'],
        'lib': ['libacados.so', 'libhpipm.so', 'libblasfeo.so', 'libformation_generator_dmpc_core.so', 'libformation_generator_dmpc_params.so', 'libformation_generator_dmpc_config.so']
    }.items():
        (out/directory).mkdir(parents=True, exist_ok=True)
        for name in names:
            shutil.copyfile(fixture, out/directory/name)
    if os.environ.get('FAKE_MISSING_PLUGIN'):
        (out/'plugins/libplan_dmpc.so').unlink()
    if os.environ.get('FAKE_MIXED_ELF'):
        shutil.copyfile(os.environ['FAKE_MIXED_ELF'], out/'plugins/libplan_dmpc.so')
    print('fixture Docker run; NOT a real compiler invocation')
else:
    print('unexpected fake docker command: ' + repr(args), file=sys.stderr)
    sys.exit(99)
'''


def run(args, env=None):
    return subprocess.run([str(x) for x in args], env=env, text=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, timeout=30)


class BuildTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.session = tempfile.TemporaryDirectory(prefix='w08-elf-')
        cls.elfs = {}
        clang = shutil.which('clang')
        if not clang or not shutil.which('ld.lld') or not shutil.which('readelf'):
            raise RuntimeError('clang, ld.lld and readelf are required; no silent fixture skips')
        source = Path(cls.session.name)/'fixture.c'
        source.write_text('int fixture_add(int a, int b) { return a+b; }\n')
        for arch, target in [('amd64', 'x86_64-linux-gnu'), ('arm64', 'aarch64-linux-gnu')]:
            elf = Path(cls.session.name)/(arch+'.so')
            result = run([clang, '--target='+target, '-fuse-ld=lld', '-nostdlib', '-shared', '-fPIC', source, '-o', elf])
            if result.returncode:
                raise RuntimeError(result.stderr)
            cls.elfs[arch] = elf
        spec = importlib.util.spec_from_file_location('onboard_elf', SCRIPTS/'verify-onboard-elf.py')
        cls.verifier = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(cls.verifier)

    @classmethod
    def tearDownClass(cls):
        cls.session.cleanup()

    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(prefix='w08 test ')
        self.addCleanup(self.temp.cleanup)
        self.base = Path(self.temp.name)
        self.project = self.base/'reviewed source'
        (self.project/'planner/formation_generator/standalone').mkdir(parents=True)
        (self.project/'planner/formation_generator/standalone/CMakeLists.txt').write_text('# test input only\n')
        (self.project/'common').mkdir()
        self.out = self.base/'exports'
        self.cache = self.base/'cache'
        self.bin = self.base/'bin'; self.bin.mkdir()
        (self.bin/'docker').write_text(FAKE_DOCKER)
        (self.bin/'docker').chmod(0o755)
        self.log = self.base/'docker.jsonl'
        self.env = {k: v for k, v in os.environ.items() if not k.startswith(('DOCKER_', 'XGC2_', 'FAKE_'))}
        self.env.update(PATH=str(self.bin)+os.pathsep+os.environ['PATH'], FAKE_LOG=str(self.log),
                        FAKE_ELF=str(self.elfs['amd64']), XGC2_BUILD_CACHE_DIR=str(self.cache))

    def args(self, arch='amd64', out=None):
        return ['bash', DRIVER, '--project', self.project, '--platform', 'focal-noetic-'+arch,
                '--output', out or self.out, '--builder-image', 'local/builder:'+arch]

    def calls(self, command=None):
        calls = [json.loads(line) for line in self.log.read_text().splitlines()] if self.log.exists() else []
        if command:
            calls = [x for x in calls if command in x]
        return calls

    def build(self, arch='amd64', **extra):
        env = dict(self.env, FAKE_ARCH=arch, FAKE_ELF=str(self.elfs[arch]), **extra)
        return run(self.args(arch), env)

    def test_shell_syntax(self):
        files = list(SCRIPTS.glob('build-onboard-*.sh'))
        if IMAGE_SCRIPT: files.append(IMAGE_SCRIPT)
        for script in files:
            result = run(['bash', '-n', script]); self.assertEqual(0, result.returncode, result.stderr)

    def test_help_without_docker(self):
        result = run(['bash', DRIVER, '--help'], self.env)
        self.assertEqual(0, result.returncode); self.assertFalse(self.log.exists())

    def test_missing_values(self):
        for flag in ['--project', '--platform', '--output', '--builder-image', '--acados-prefix', '--jobs']:
            result = run(['bash', DRIVER, flag], self.env)
            self.assertEqual(2, result.returncode); self.assertIn('requires a value', result.stderr)
        self.assertFalse(self.log.exists())

    def test_unknown_platform(self):
        result = run(self.args('robot-01')+['--dry-run'], self.env)
        self.assertEqual(2, result.returncode); self.assertIn('unsupported target', result.stderr)

    def test_bad_jobs(self):
        for value in ['0', '-1', 'abc']:
            result = run(self.args()+['--jobs', value], self.env)
            self.assertEqual(2, result.returncode)
        self.assertFalse(self.log.exists())

    def test_builder_required(self):
        result = run(self.args()[:-2], self.env)
        self.assertEqual(2, result.returncode); self.assertIn('locally prepared', result.stderr)

    def test_missing_project(self):
        shutil.rmtree(self.project/'common')
        result = run(self.args(), self.env)
        self.assertEqual(2, result.returncode); self.assertIn('--project', result.stderr)

    def test_dry_run_is_read_only(self):
        for arch in ['amd64', 'arm64']:
            result = run(self.args(arch)+['--dry-run'], self.env)
            self.assertEqual(0, result.returncode, result.stderr)
            self.assertIn('DOCKER_PLATFORM=linux/'+arch, result.stdout)
        self.assertFalse(self.out.exists()); self.assertFalse(self.cache.exists()); self.assertFalse(self.log.exists())

    def test_output_cannot_overlap_source(self):
        result = run(self.args(out=self.project/'out'), self.env)
        self.assertEqual(2, result.returncode); self.assertFalse(self.log.exists())

    def test_mount_separator_rejected(self):
        result = run(self.args(out=self.base/'bad,out'), self.env)
        self.assertEqual(2, result.returncode); self.assertIn('commas', result.stderr)

    def test_remote_host_rejected_before_inspection(self):
        result = run(self.args(), dict(self.env, DOCKER_HOST='ssh://robot.invalid'))
        self.assertEqual(2, result.returncode); self.assertFalse(self.log.exists())

    def test_remote_context_rejected(self):
        result = run(self.args(), dict(self.env, FAKE_ENDPOINT='tcp://example.invalid:2375'))
        self.assertEqual(2, result.returncode); self.assertEqual(1, len(self.calls()))

    def test_selected_context_wins_over_host(self):
        result = run(self.args(), dict(self.env, DOCKER_HOST='tcp://example.invalid:2375', DOCKER_CONTEXT='local-fixture'))
        self.assertEqual(0, result.returncode, result.stderr)
        self.assertIn('unix:///tmp/test-docker.sock', self.calls('run')[0])

    def test_wrong_image_architecture(self):
        result = run(self.args(), dict(self.env, FAKE_ARCH='arm64'))
        self.assertEqual(2, result.returncode); self.assertIn('builder architecture', result.stderr)
        self.assertFalse(self.calls('run'))

    def test_missing_local_image_does_not_pull(self):
        result = run(self.args(), dict(self.env, FAKE_MISSING_IMAGE='1'))
        self.assertEqual(9, result.returncode); self.assertFalse(self.calls('pull')); self.assertFalse(self.calls('run'))

    def test_compiled_elf_identity_for_both_targets(self):
        for arch in ['amd64', 'arm64']:
            self.assertIn('ELF64', self.verifier.verify(self.elfs[arch], 'focal-noetic-'+arch))
            other = 'arm64' if arch == 'amd64' else 'amd64'
            with self.assertRaisesRegex(ValueError, 'wrong ELF'):
                self.verifier.verify(self.elfs[arch], 'focal-noetic-'+other)

    def test_non_elf_and_truncated_input_rejected(self):
        for data in [b'#!/bin/sh\nexit 0\n', b'\x7fELF\x02\x01\x01']:
            path = self.base/'bad.so'; path.write_bytes(data)
            with self.assertRaisesRegex(ValueError, 'not a 64-bit'):
                self.verifier.verify(path, 'focal-noetic-amd64')

    def test_escaped_symlink_rejected(self):
        path = self.base/'escape.so'; path.symlink_to(self.elfs['amd64'])
        with self.assertRaisesRegex(ValueError, 'escapes'):
            self.verifier.verify(path, 'focal-noetic-amd64')

    def test_same_directory_symlink_allowed(self):
        shutil.copyfile(self.elfs['amd64'], self.base/'real.so.1')
        path = self.base/'alias.so'; path.symlink_to('real.so.1')
        self.assertIn('machine=62', self.verifier.verify(path, 'focal-noetic-amd64'))

    def test_fixture_docker_export_for_both_targets(self):
        for arch in ['amd64', 'arm64']:
            result = self.build(arch)
            self.assertEqual(0, result.returncode, result.stderr)
            target = self.out/('focal-noetic-'+arch)
            self.assertTrue((target/'bin/xgc-rt-render').is_file())
            self.assertIn('artifact_exit_code=0', (target/'timing.env').read_text())
            self.assertNotIn(str(self.out), (target/'ELF.txt').read_text())
            self.assertFalse((target/'BUNDLE.json').exists())
        call = self.calls('run')[0]
        self.assertIn('--pull=never', call); self.assertNotIn('--privileged', call)
        self.assertEqual('none', call[call.index('--network')+1])
        self.assertIn('--user', call); self.assertIn('--cap-drop', call)
        self.assertTrue(any('dst=/workspace/project,readonly' in arg for arg in call))
        self.assertTrue(any('dst=/workspace/runtime,readonly' in arg for arg in call))
        self.assertFalse(any('/var/run/docker.sock' in arg for arg in call))

    def test_adjacent_workspace_tests_are_read_only(self):
        (self.project.parent/'tests').mkdir()
        result = self.build()
        self.assertEqual(0, result.returncode, result.stderr)
        call = self.calls('run')[0]
        self.assertTrue(any('dst=/workspace/tests,readonly' in arg for arg in call))

    def test_failure_retains_logs_without_final_output(self):
        result = self.build(FAKE_BUILD_RC='17')
        self.assertEqual(17, result.returncode)
        self.assertFalse((self.out/'focal-noetic-amd64').exists())
        stage = next(self.out.glob('.focal-noetic-amd64.*'))
        self.assertIn('compiler/linker failed', (stage/'build.log').read_text())
        self.assertIn('docker_exit_code=17', (stage/'timing.env').read_text())

    def test_docker_zero_without_artifacts_is_not_success(self):
        result = self.build(FAKE_EMPTY_OUTPUT='1')
        self.assertEqual(4, result.returncode); self.assertFalse((self.out/'focal-noetic-amd64').exists())

    def test_mixed_architecture_export_is_rejected(self):
        result = self.build(FAKE_MIXED_ELF=str(self.elfs['arm64']))
        self.assertEqual(4, result.returncode); self.assertIn('wrong ELF', result.stderr)
        stage = next(self.out.glob('.focal-noetic-amd64.*'))
        self.assertIn('artifact_exit_code=4', (stage/'timing.env').read_text())

    def test_missing_required_plugin_is_rejected(self):
        result = self.build(FAKE_MISSING_PLUGIN='1')
        self.assertEqual(4, result.returncode)
        self.assertIn('missing plugins/libplan_dmpc.so', result.stderr)
        self.assertFalse((self.out/'focal-noetic-amd64').exists())

    def test_source_revision_and_dirty_state_are_recorded(self):
        for command in [['git', 'init', self.project], ['git', '-C', self.project, 'add', '.'],
                        ['git', '-C', self.project, '-c', 'user.name=W08 Test', '-c', 'user.email=w08@example.invalid', 'commit', '-m', 'fixture']]:
            result = run(command, self.env)
            self.assertEqual(0, result.returncode, result.stderr)
        revision = run(['git', '-C', self.project, 'rev-parse', 'HEAD']).stdout.strip()
        result = self.build()
        self.assertEqual(0, result.returncode, result.stderr)
        identity = (self.out/'focal-noetic-amd64/build-target.env').read_text()
        self.assertIn('PROJECT_REVISION='+revision, identity)
        self.assertIn('PROJECT_WORKTREE=clean', identity)
        (self.project/'planner/formation_generator/standalone/CMakeLists.txt').write_text('# edited fixture\n')
        dirty_out = self.base/'dirty export'
        result = run(self.args(out=dirty_out), self.env)
        self.assertEqual(0, result.returncode, result.stderr)
        identity = (dirty_out/'focal-noetic-amd64/build-target.env').read_text()
        self.assertIn('PROJECT_REVISION='+revision, identity)
        self.assertIn('PROJECT_WORKTREE=dirty', identity)

    def test_existing_output_preserved(self):
        target = self.out/'focal-noetic-amd64'; target.mkdir(parents=True)
        (target/'sentinel').write_text('keep')
        result = self.build()
        self.assertEqual(2, result.returncode); self.assertEqual('keep', (target/'sentinel').read_text())
        self.assertFalse(self.log.exists())

    def test_cache_mount_reuse_not_a_cache_performance_claim(self):
        for out in [self.out, self.base/'second export']:
            result = run(self.args(out=out), self.env)
            self.assertEqual(0, result.returncode, result.stderr)
        mounts = [next(x for x in call if 'dst=/workspace/cache' in x) for call in self.calls('run')]
        self.assertEqual(mounts[0], mounts[1])
        result = run(self.args(out=self.base/'new image'), dict(self.env, FAKE_IMAGE_ID='b'*64))
        self.assertEqual(0, result.returncode, result.stderr)
        new_mount = next(x for x in self.calls('run')[-1] if 'dst=/workspace/cache' in x)
        self.assertNotEqual(mounts[0], new_mount)

    @unittest.skipUnless(IMAGE_SCRIPT, 'set XGC2_IMAGES_SOURCE for the W08 image PR checks')
    def test_image_platform_tags_and_legacy_default(self):
        for args, arch, tag in [([], 'amd64', ':base-local'), (['linux/amd64'], 'amd64', ':base-local-amd64'), (['linux/arm64'], 'arm64', ':base-local-arm64')]:
            result = run(['bash', IMAGE_SCRIPT, 'scout-focal-noetic']+args, dict(self.env, FAKE_ARCH=arch))
            self.assertEqual(0, result.returncode, result.stderr)
            call = self.calls('build')[-1]
            self.assertEqual('linux/'+arch, call[call.index('--platform')+1])
            self.assertTrue(call[call.index('-t')+1].endswith(tag))
            self.assertNotIn('--push', call)
            self.assertFalse(any(arg.startswith('PARENT_IMAGE=') for arg in call))

    @unittest.skipUnless(IMAGE_SCRIPT, 'set XGC2_IMAGES_SOURCE for the W08 image PR checks')
    def test_image_argument_and_unvalidated_sitl_rejection(self):
        for args in [['scout-focal-noetic', 'linux/riscv64'], ['unknown'], ['fs150-focal-noetic-sitl', 'linux/arm64'], ['scout-focal-noetic', 'linux/amd64', 'extra']]:
            result = run(['bash', IMAGE_SCRIPT]+args, self.env)
            self.assertEqual(2, result.returncode)
        self.assertFalse(self.log.exists())

    @unittest.skipUnless(IMAGE_SCRIPT, 'set XGC2_IMAGES_SOURCE for the W08 image PR checks')
    def test_sitl_default_parent_matches_explicit_architecture_tag(self):
        result = run(['bash', IMAGE_SCRIPT, 'fs150-focal-noetic-sitl', 'linux/amd64'], self.env)
        self.assertEqual(0, result.returncode, result.stderr)
        call = self.calls('build')[0]
        self.assertIn('PARENT_IMAGE=onboard-sim-fs150-focal-noetic:base-local-amd64', call)
        self.assertEqual('onboard-sim-fs150-focal-noetic:base-local-amd64-sitl-1.1.0-23', call[call.index('-t')+1])

    @unittest.skipUnless(IMAGE_SCRIPT, 'set XGC2_IMAGES_SOURCE for the W08 image PR checks')
    def test_sitl_parent_architecture_checked_before_build(self):
        result = run(['bash', IMAGE_SCRIPT, 'fs150-focal-noetic-sitl', 'linux/amd64'], dict(self.env, FAKE_ARCH='arm64'))
        self.assertEqual(1, result.returncode); self.assertFalse(self.calls('build'))

    @unittest.skipUnless(IMAGE_SCRIPT, 'set XGC2_IMAGES_SOURCE for the W08 image PR checks')
    def test_image_result_architecture_checked(self):
        result = run(['bash', IMAGE_SCRIPT, 'scout-focal-noetic', 'linux/arm64'], self.env)
        self.assertEqual(1, result.returncode); self.assertIn('expected linux/arm64', result.stderr)


if __name__ == '__main__':
    unittest.main(verbosity=2)
