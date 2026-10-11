"""다중 드론 스폰: 드론 i마다 SDF를 생성(models/generated/drone_i/model.sdf)하고 /world/<world>/create 로 스폰한다.

사용: python3 spawn_drones.py 0 1 [--world room] [--pos 1=3,-2[,z[,yaw]]] [--gen-only]
드론 i = 모델 drone_i, ArduPilot 플러그인 fdm_port_in=9002+10i, 센서 토픽 /drone_i/{front_camera/image,down_camera/image,down_range/scan}.
센서 설정은 sensors/<이름>/model.sdf 에서 읽어 오므로 그쪽을 바꾸면 다음 스폰부터 반영된다.
"""
import argparse, math, os, re, sys
import xml.etree.ElementTree as ET

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
GEN_DIR = os.path.join(ROOT, "models", "generated")
SPAWN_Z = 0.2
LIMIT = 5.5  # 벽(±6m)에서 0.5m 안쪽
# (sensors/ 모델, 본체 기준 장착 pose "x y z r p y", 센서 토픽)
SENSORS = [
    ("front_camera", "0.1 0 0.05 0 0 0", "front_camera/image"),  # docs/front_camera_spec.md, get_truth.CAMERA_OFFSET
    ("down_camera", "0 0 -0.05 0 0 0", "down_camera/image"),
    ("down_range", "0 0 -0.05 0 0 0", "down_range/scan"),
]


def default_pos(i):
    """0:(-2,0) 1:(2,0) (A 가이드). 이후 2대씩 행을 늘린다: 행 r=i//2, y=0,+3,-3 (x=-2/2). i>=6 은 --pos 필수."""
    if i >= 6:
        raise ValueError(f"드론 {i}: 기본 배치는 0..5 까지. --pos {i}=x,y 로 직접 지정")
    r = i // 2
    return (-2.0 if i % 2 == 0 else 2.0), 3.0 * ((r + 1) // 2) * (1 if r % 2 else -1), SPAWN_Z, 0.0


def find_resource(rel):
    for d in os.environ.get("GZ_SIM_RESOURCE_PATH", "").split(":") + [os.path.join(ROOT, "sensors")]:
        p = os.path.join(d, rel)
        if d and os.path.isfile(p):
            return p
    raise FileNotFoundError(f"{rel} 를 GZ_SIM_RESOURCE_PATH 에서 못 찾음 (source env.sh 했는지 확인)")


def gen_sdf(i):
    """원본 iris_with_ardupilot 를 복사해 모델 이름/포트를 바꾸고 센서 링크+고정 조인트를 붙인다. 파일 경로를 돌려준다."""
    s = open(find_resource("iris_with_ardupilot/model.sdf"), encoding="utf-8").read()
    s = re.sub(r"<!--.*?-->", "", s, flags=re.S)
    s, n1 = re.subn(r'<model name="iris_with_ardupilot">', f'<model name="drone_{i}">', s)
    s, n2 = re.subn(r"<fdm_port_in>\d+</fdm_port_in>", f"<fdm_port_in>{9002 + 10 * i}</fdm_port_in>", s)
    assert n1 == 1 and n2 == 1, "원본 iris 구조가 예상과 다름"
    extra = ""
    for name, pose, topic in SENSORS:
        link = ET.parse(find_resource(f"{name}/model.sdf")).getroot().find("model/link")
        link.set("name", f"{name}_link")
        link.insert(0, ET.Element("pose"))
        link[0].text = pose
        link.find("sensor/topic").text = f"/drone_{i}/{topic}"
        extra += ET.tostring(link, encoding="unicode") + "\n"
        extra += (f'<joint name="{name}_joint" type="fixed"><parent>iris_with_standoffs::base_link</parent>'
                  f"<child>{name}_link</child></joint>\n")
    k = s.rindex("</model>")
    s = s[:k] + extra + s[k:]
    d = os.path.join(GEN_DIR, f"drone_{i}")
    os.makedirs(d, exist_ok=True)
    path = os.path.join(d, "model.sdf")
    open(path, "w", encoding="utf-8").write(s)
    return path


def spawn(i, world, pos):
    from gz.transport13 import Node
    from gz.msgs10.entity_factory_pb2 import EntityFactory
    from gz.msgs10.boolean_pb2 import Boolean
    x, y, z, yaw = pos
    req = EntityFactory(sdf_filename=gen_sdf(i), name=f"drone_{i}", allow_renaming=False)
    req.pose.position.x, req.pose.position.y, req.pose.position.z = x, y, z
    req.pose.orientation.z, req.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
    ok, rep = Node().request(f"/world/{world}/create", req, EntityFactory, Boolean, 10000)
    return ok and rep.data


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("ids", type=int, nargs="+")
    ap.add_argument("--world", default="room")
    ap.add_argument("--pos", action="append", default=[], metavar="ID=x,y[,z[,yaw]]", help="ENU m, yaw rad")
    ap.add_argument("--gen-only", action="store_true", help="SDF만 생성하고 스폰하지 않음")
    a = ap.parse_args()
    over = {}
    for p in a.pos:
        k, v = p.split("=")
        f = [float(t) for t in v.split(",")]
        over[int(k)] = (f + [SPAWN_Z, 0.0][len(f) - 2:])[:4]
    rc = 0
    for i in a.ids:
        x, y, z, yaw = over.get(i) or default_pos(i)
        if abs(x) > LIMIT or abs(y) > LIMIT:
            sys.exit(f"드론 {i}: ({x},{y}) 가 벽 ±{LIMIT}m 밖")
        if a.gen_only:
            print(f"drone_{i}: {gen_sdf(i)}")
            continue
        ok = spawn(i, a.world, (x, y, z, yaw))
        print(f"drone_{i} at ({x},{y},{z}) yaw {yaw}: {'OK' if ok else 'FAIL'}  fdm_port_in={9002 + 10 * i}")
        rc |= not ok
    sys.exit(rc)


if __name__ == "__main__":
    main()
