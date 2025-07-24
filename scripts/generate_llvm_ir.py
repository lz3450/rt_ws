import os
import json
import subprocess

os.makedirs('ir', exist_ok=True)

with open(file='compile_commands.json', mode='r', encoding='utf-8') as f:
    compile_cmds: list[dict[str, str]] = json.load(f)
    for cmd in compile_cmds:
        params = cmd['command'].split()
        o_index = params.index('-o')
        del params[o_index:o_index+2]
        params.extend(['-S', '-emit-llvm', '-fno-discard-value-names', '-Xclang', '-disable-O0-optnone'])
        params.extend(['-o', f'/home/kzl/ros2_ws/src/turtle_tf2_cpp/ir/{os.path.basename(cmd["file"])}.ll', '-c', cmd['file']])
        # subprocess.run(params, stdout=subprocess.PIPE, check=True)
