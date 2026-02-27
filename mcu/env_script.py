Import("env")
import os

# PROJECT_DIR 기준으로 상위 디렉토리(mcu/)에 위치한 .env 파일 탐색
project_dir = env.get("PROJECT_DIR")
env_file = os.path.join(project_dir, "..", ".env")
env_file = os.path.abspath(env_file)

print(f"[env_script] Looking for .env at: {env_file}")

if os.path.exists(env_file):
    print(f"[env_script] Found .env, loading variables...")
    with open(env_file, "r") as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#") or "=" not in line:
                continue

            key, val = line.split("=", 1)
            key = key.strip()
            val = val.strip().strip('"').strip("'")

            if not key or not val:
                continue

            # -D KEY=\"val\" 형태로 build_flags에 직접 추가
            flag = f'-D{key}=\\"{val}\\"'
            env.Append(BUILD_FLAGS=[flag])
            print(f"[env_script]   {key} = {val}")
else:
    print(f"[env_script] WARNING: No .env file found at {env_file}")
