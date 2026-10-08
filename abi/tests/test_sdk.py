#!/usr/bin/env python3
"""Validate the SDK using explicit source, installed, relocated and deb inputs.

Needs CMake >= 3.16, C/C++ compilers and dpkg-deb; performs no global install.
All build/install outputs stay under the required fresh --work-dir.
"""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import subprocess


C_SMOKE = r'''
#include <stddef.h>
#include <xgc_rt.h>
#include <xgc_clock_source.h>
_Static_assert(XGC_RT_ABI_VERSION == 1 && XGC_RT_ABI_MINOR == 3, "plugin ABI");
_Static_assert(XGC_RT_MAX_PORTS == 64, "port capacity");
_Static_assert(sizeof(xgc_step_ctx) == 48, "step layout");
_Static_assert(offsetof(xgc_step_ctx, dirty_ports) == 32, "step mask offset");
_Static_assert(sizeof(xgc_plugin_descriptor) == 40, "64-bit descriptor");
_Static_assert(sizeof(xgc_clock_observation_v1) == 544, "clock observation");
_Static_assert(offsetof(xgc_clock_observation_v1, publisher) == 32, "clock publisher");
_Static_assert(XGC_CLOCK_SOURCE_ABI_VERSION == 1, "clock ABI");
int main(void) { return XGC_OK; }
'''

CXX_SMOKE = r'''
#include <cstddef>
#include <type_traits>
#include <xgc_rt.h>
#include <xgc_clock_source.h>
#include <flat_config.hpp>
static_assert(std::is_standard_layout<xgc_host_api>::value, "host API layout");
static_assert(sizeof(xgc_step_ctx) == 48, "step layout");
static_assert(sizeof(xgc_clock_source_descriptor_v1) == 16, "clock descriptor");
int main() {
  int rate = 0; bool enabled = false;
  const std::string config = "rate = 5\nenabled = true\nname = \"SDK\"\n";
  return !(xgc_rt_config::integer(config, "rate", &rate) && rate == 5 &&
           xgc_rt_config::boolean(config, "enabled", &enabled) && enabled &&
           xgc_rt_config::text_or(config, "name", "missing") == "SDK");
}
'''

CONSUMER = r'''
cmake_minimum_required(VERSION 3.16)
project(RuntimeSDKConsumer LANGUAGES C CXX)
if(DEFINED XGC_RUNTIME_SDK_SOURCE_ROOT)
  # Explicit developer source mode, never an installed-package fallback.
  add_subdirectory("${XGC_RUNTIME_SDK_SOURCE_ROOT}/abi" sdk)
else()
  find_package(XgcRuntimeSDK ${SDK_VERSION} EXACT CONFIG REQUIRED
    PATHS "${SDK_PREFIX}/share/cmake/XgcRuntimeSDK" NO_DEFAULT_PATH)
endif()
if(NOT TARGET XgcRuntime::SDK)
  message(FATAL_ERROR "missing documented SDK target")
endif()
get_target_property(sdk_kind XgcRuntime::SDK TYPE)
if(NOT sdk_kind STREQUAL "INTERFACE_LIBRARY")
  message(FATAL_ERROR "SDK must remain a header-only INTERFACE target")
endif()
get_target_property(sdk_links XgcRuntime::SDK INTERFACE_LINK_LIBRARIES)
if(sdk_links)
  message(FATAL_ERROR "SDK unexpectedly brings runtime/domain libraries: ${sdk_links}")
endif()
if(DEFINED SDK_RETIRED_HEADER)
  file(WRITE "${CMAKE_CURRENT_BINARY_DIR}/retired.c" "#include <${SDK_RETIRED_HEADER}>\nint main(void) { return 0; }\n")
  add_executable(retired_header "${CMAKE_CURRENT_BINARY_DIR}/retired.c")
  target_link_libraries(retired_header PRIVATE XgcRuntime::SDK)
  return()
endif()
add_executable(c_smoke c_smoke.c)
set_target_properties(c_smoke PROPERTIES C_STANDARD 11 C_STANDARD_REQUIRED YES C_EXTENSIONS NO)
target_link_libraries(c_smoke PRIVATE XgcRuntime::SDK)
add_executable(cxx_smoke cxx_smoke.cpp)
set_target_properties(cxx_smoke PROPERTIES CXX_STANDARD 11 CXX_STANDARD_REQUIRED YES CXX_EXTENSIONS NO)
target_link_libraries(cxx_smoke PRIVATE XgcRuntime::SDK)
'''


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--work-dir', required=True, type=Path)
    args = parser.parse_args()
    source = Path(__file__).resolve().parents[2]
    work = args.work_dir.resolve()
    work.mkdir(parents=True, exist_ok=False)
    log = work / 'commands.log'
    checks = []

    def run(*command, success=True):
        command = [str(item) for item in command]
        result = subprocess.run(command, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
        with log.open('a') as handle:
            handle.write(json.dumps(command) + '\n' + result.stdout + '\n')
        if (result.returncode == 0) != success:
            raise AssertionError('unexpected exit {}: {}\n{}'.format(result.returncode, command, result.stdout))
        return result.stdout

    consumer = work / 'consumer'
    consumer.mkdir()
    (consumer / 'CMakeLists.txt').write_text(CONSUMER)
    (consumer / 'c_smoke.c').write_text(C_SMOKE)
    (consumer / 'cxx_smoke.cpp').write_text(CXX_SMOKE)

    def consume(name, prefix=None, source_mode=False):
        build = work / ('consumer-' + name)
        settings = ['-DXGC_RUNTIME_SDK_SOURCE_ROOT=' + str(source)] if source_mode else [
            '-DSDK_PREFIX=' + str(prefix), '-DSDK_VERSION=0.1.0']
        run('cmake', '-S', consumer, '-B', build, *settings)
        run('cmake', '--build', build, '--parallel', '1')
        run(build / 'c_smoke')
        run(build / 'cxx_smoke')
        checks.append(name + ': C11/C++11 generic Host/clock ABI/config smoke passed')
        for header in ('xgc_schemas_v1.h', 'xgc_dmpc_planner_v1.h'):
            negative = work / ('negative-' + name + '-' + header)
            run('cmake', '-S', consumer, '-B', negative, *settings, '-DSDK_RETIRED_HEADER=' + header)
            run('cmake', '--build', negative, '--parallel', '1', success=False)
        checks.append(name + ': retired domain headers refused through SDK-only target')

    consume('source', source_mode=True)
    installed = work / 'installed'
    run('cmake', '-S', source / 'abi', '-B', work / 'sdk-build', '-DCMAKE_INSTALL_PREFIX=' + str(installed))
    run('cmake', '--install', work / 'sdk-build')
    assert {path.name for path in (source / 'abi/include').glob('*.h')} == {'xgc_rt.h', 'xgc_clock_source.h'}, 'domain source headers must be retired'
    canonical = {name: source / 'abi/include' / name for name in ('xgc_rt.h', 'xgc_clock_source.h')}
    canonical['flat_config.hpp'] = source / 'plugins/common/flat_config.hpp'
    assert set(canonical) == {'xgc_rt.h', 'xgc_clock_source.h', 'flat_config.hpp'}

    def check_payload(prefix):
        headers = prefix / 'include/xgc-runtime'
        assert {path.name for path in headers.iterdir()} == set(canonical)
        assert not (headers / 'xgc_dmpc_planner_v1.h').exists()
        assert not (headers / 'xgc_schemas_v1.h').exists()
        for name, original in canonical.items():
            assert (headers / name).read_bytes() == original.read_bytes(), name + ' differs from its canonical source'
        for config in (prefix / 'share/cmake/XgcRuntimeSDK').glob('*.cmake'):
            assert str(source) not in config.read_text(), str(config) + ' leaks a source include path'
        assert not list(prefix.rglob('*.so')) and not (prefix / 'bin').exists()

    check_payload(installed)
    consume('installed', installed)
    relocated = work / 'relocated'
    installed.rename(relocated)
    consume('relocated', relocated)
    run('cmake', '-S', consumer, '-B', work / 'wrong-version', '-DSDK_PREFIX=' + str(relocated), '-DSDK_VERSION=99.0.0', success=False)
    checks.append('unavailable SDK version refused')

    deb_dir = work / 'debs'
    run('bash', source / '.xgc2/scripts/build_deb.sh', '--output', deb_dir)
    packages = list(deb_dir.glob('*.deb'))
    assert len(packages) == 1
    deb = packages[0]
    fields = {}
    for field in ['Package', 'Version', 'Architecture', 'Depends']:
        fields[field] = run('dpkg-deb', '-f', deb, field).strip()
    expected_version = next(line.split(':', 1)[1].strip()
                            for line in (source / '.xgc2/product.yml').read_text().splitlines()
                            if line.strip().startswith('focal:'))
    assert fields == {'Package': 'libxgc2-runtime-sdk-dev', 'Version': expected_version, 'Architecture': 'all', 'Depends': ''}, fields
    before = hashlib.sha256(deb.read_bytes()).hexdigest()
    run('bash', source / '.xgc2/scripts/build_deb.sh', '--output', deb_dir, success=False)
    assert hashlib.sha256(deb.read_bytes()).hexdigest() == before
    extract = work / 'deb-extracted'
    run('dpkg-deb', '-x', deb, extract)
    check_payload(extract / 'usr')
    consume('deb', extract / 'usr')
    checks.append('one all-architecture deb, no runtime Depends, existing artifact preserved')
    receipt = {'checks': checks, 'package_fields': fields, 'deb': str(deb), 'deb_sha256': before,
               'canonical_sha256': {name: hashlib.sha256(path.read_bytes()).hexdigest() for name, path in canonical.items()},
               'scope': 'local source/private-prefix/deb-extract only; no APT publication or owner-package installation claim'}
    (work / 'result.json').write_text(json.dumps(receipt, indent=2) + '\n')
    print(json.dumps(receipt, indent=2))


if __name__ == '__main__':
    main()
