#!/usr/bin/env python3
"""Check the one SDK Deb after dpkg installation; never rebuild package bytes."""
import argparse
import hashlib
import importlib.util
import json
from pathlib import Path
import re
import shutil
import subprocess


def require(condition, message):
    if not condition:
        raise RuntimeError(message)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--deb', required=True, type=Path)
    parser.add_argument('--work-dir', required=True, type=Path)
    args = parser.parse_args()
    source = Path(__file__).resolve().parents[2]
    work = args.work_dir.resolve()
    work.mkdir(parents=True, exist_ok=False)
    checks = []

    def run(*command, success=True):
        command = [str(item) for item in command]
        result = subprocess.run(command, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
        with (work / 'commands.log').open('a') as log:
            log.write(json.dumps(command) + '\n' + result.stdout + '\n')
        require((result.returncode == 0) == success,
                'unexpected exit {}: {}\n{}'.format(result.returncode, command, result.stdout))
        return result.stdout.strip()

    metadata = (source / '.xgc2/product.yml').read_text()
    version = re.search(r'^    focal: (\S+)$', metadata, re.M).group(1)
    package = 'libxgc2-runtime-sdk-dev'
    fields = {field: run('dpkg-deb', '-f', args.deb, field)
              for field in ['Package', 'Version', 'Architecture', 'Depends']}
    require(fields == {'Package': package, 'Version': version, 'Architecture': 'all', 'Depends': ''},
            'unexpected SDK package fields: ' + str(fields))
    require(run('dpkg-query', '-W', '-f=${Status}', package) == 'install ok installed', 'SDK not installed')
    require(run('dpkg-query', '-W', '-f=${Version}', package) == version, 'installed SDK version mismatch')
    spec = importlib.util.spec_from_file_location('owning_sdk_tests', source / 'abi/tests/test_sdk.py')
    tests = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(tests)
    consumer = work / 'consumer'
    consumer.mkdir()
    (consumer / 'CMakeLists.txt').write_text(tests.CONSUMER)
    (consumer / 'c_smoke.c').write_text(tests.C_SMOKE)
    (consumer / 'cxx_smoke.cpp').write_text(tests.CXX_SMOKE)
    canonical = {path.name: path for path in (source / 'abi/include').glob('*.h')}
    canonical['flat_config.hpp'] = source / 'plugins/common/flat_config.hpp'
    require(set(canonical) == {'xgc_rt.h', 'xgc_clock_source.h', 'xgc_schemas_v1.h',
                               'xgc_dmpc_planner_v1.h', 'flat_config.hpp'}, 'unexpected canonical SDK headers')
    config_names = {'XgcRuntimeSDKConfig.cmake', 'XgcRuntimeSDKConfigVersion.cmake', 'XgcRuntimeSDKTargets.cmake'}

    def payload(prefix):
        headers = prefix / 'include/xgc-runtime'
        require({path.name for path in headers.iterdir()} == set(canonical), 'SDK header set mismatch')
        for name, original in canonical.items():
            require((headers / name).read_bytes() == original.read_bytes(), 'noncanonical SDK header: ' + name)
        configs = prefix / 'share/cmake/XgcRuntimeSDK'
        require({path.name for path in configs.iterdir()} == config_names, 'SDK CMake package set mismatch')
        for path in configs.iterdir():
            require(str(source) not in path.read_text(), 'installed config leaks source path')

    def consume(name, prefix=None, source_mode=False, build_success=True):
        build = work / name
        settings = ['-DXGC_RUNTIME_SDK_SOURCE_ROOT=' + str(source)] if source_mode else [
            '-DSDK_PREFIX=' + str(prefix), '-DSDK_VERSION=0.1.0']
        run('cmake', '-S', consumer, '-B', build, *settings)
        run('cmake', '--build', build, '--parallel', '1', success=build_success)
        if build_success:
            run(build / 'c_smoke')
            run(build / 'cxx_smoke')

    extracted = work / 'extracted'
    run('dpkg-deb', '-x', args.deb, extracted)
    expected = {'usr/include/xgc-runtime/' + name for name in canonical}
    expected.update('usr/share/cmake/XgcRuntimeSDK/' + name for name in config_names)
    actual = {path.relative_to(extracted).as_posix() for path in extracted.rglob('*') if not path.is_dir()}
    require(actual == expected, 'SDK Deb contains missing or extra payload: ' + str(actual ^ expected))
    payload(extracted / 'usr')
    payload(Path('/usr'))
    for path in sorted(expected):
        owner = run('dpkg-query', '-S', '/' + path)
        require(owner.startswith(package + ': '), 'SDK install ownership mismatch: ' + owner)
    consume('source', source_mode=True)
    consume('installed', Path('/usr'))
    relocated = work / 'relocated'
    shutil.copytree(extracted / 'usr', relocated)
    consume('relocated', relocated)
    run('cmake', '-S', consumer, '-B', work / 'wrong-version',
        '-DSDK_PREFIX=' + str(relocated), '-DSDK_VERSION=99.0.0', success=False)
    for name in sorted(canonical):
        header = relocated / 'include/xgc-runtime' / name
        original = header.read_bytes()
        header.unlink()
        try:
            consume('missing-' + name.replace('.', '-'), relocated, build_success=False)
        finally:
            header.write_bytes(original)
    config = relocated / 'share/cmake/XgcRuntimeSDK/XgcRuntimeSDKConfig.cmake'
    config.unlink()
    run('cmake', '-S', consumer, '-B', work / 'missing-config',
        '-DSDK_PREFIX=' + str(relocated), '-DSDK_VERSION=0.1.0', success=False)
    checks.extend(['canonical header-only Deb payload and installed dpkg ownership',
                   'source/installed/relocated XgcRuntime::SDK C11/C++11 ABI and config smoke',
                   'wrong SDK version, each missing header and missing package config refused'])
    print(json.dumps({'checks': checks, 'package_fields': fields,
                      'deb_sha256': hashlib.sha256(args.deb.read_bytes()).hexdigest(),
                      'architecture': run('dpkg', '--print-architecture')}, indent=2))


if __name__ == '__main__':
    main()
