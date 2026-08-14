"""
robot_draft.xml에 4개 크랭크-슬라이더 기구(좌우 hip, 좌우 ankle)의
slot_follower body + site + equality connect를 추가하는 스크립트.

측정 데이터 출처: 사용자가 CAD에서 직접 측정 (mm 단위)
  - 크랭크 핀 site 위치: 각 크랭크 링크 자신의 로컬 좌표계 기준
  - slot_follower body pos: 각 부모(슬라이더/ankle) 링크 자신의 로컬 좌표계 기준
  - 슬롯 axis/range: 슬라이더 부품 파일(mesh 로컬 좌표계, rpy=0 확인됨) 기준,
    4개 슬라이더 전부 동일 부품이라 공유
"""
import xml.etree.ElementTree as ET

DRAFT_PATH = "/home/geonhu/colcon_ws/src/kid_RL_v2_description/urdf/robot_draft.xml"

MM = 0.001

# 크랭크 핀 site 위치 (크랭크 링크 자신의 로컬 좌표계, mm)
CRANK_PIN_MM = {
    "left_hip_crank_1": (5, -0.096, -20),
    "right_hip_crank_1": (5, -0.095, -20),
    "left_ankle_crank_1": (5, 0.095, 20),
    "right_ankle_crank_1": (5, 0.096, 20),
}

# slot_follower body pos (부모 링크 자신의 로컬 좌표계, mm)
FOLLOWER_PARENT_POS_MM = {
    "left_hip_slider_1": (-4.44, -0.095, -40),
    "right_hip_slider_1": (-4.44, -0.095, -40),
    "left_ankle_1": (-32.8, 24.603, 40),
    "right_ankle_1": (-32.8, -24.413, 40),
}

# (mechanism_name, crank_body, parent_body) — 4개 기구
MECHANISMS = [
    ("left_hip", "left_hip_crank_1", "left_hip_slider_1"),
    ("right_hip", "right_hip_crank_1", "right_hip_slider_1"),
    ("left_ankle", "left_ankle_crank_1", "left_ankle_1"),
    ("right_ankle", "right_ankle_crank_1", "right_ankle_1"),
]

# 슬롯 axis/range — 4개 슬라이더가 전부 같은 부품 파일이라 공유
AXIS = "-0.02 0 0"
RANGE = "0 0.02"

FOLLOWER_MASS = "0.004"
FOLLOWER_INERTIA = "1e-9 1e-9 1e-9"


def fmt(v):
    return f"{v[0]*MM:.6f} {v[1]*MM:.6f} {v[2]*MM:.6f}"


def find_body(root, name):
    for b in root.iter("body"):
        if b.get("name") == name:
            return b
    raise ValueError(f"body '{name}' not found")


def main():
    tree = ET.parse(DRAFT_PATH)
    root = tree.getroot()

    equality = root.find("equality")
    if equality is None:
        equality = ET.SubElement(root, "equality")

    for mech_name, crank_name, parent_name in MECHANISMS:
        crank_body = find_body(root, crank_name)
        parent_body = find_body(root, parent_name)

        # 1) 크랭크 body에 pin site 추가
        pin_site_name = f"{mech_name}_crank_pin_site"
        pin_site = ET.SubElement(crank_body, "site")
        pin_site.set("name", pin_site_name)
        pin_site.set("pos", fmt(CRANK_PIN_MM[crank_name]))

        # 2) 부모 body 안에 slot_follower body 추가
        follower_body_name = f"{mech_name}_slot_follower"
        follower_body = ET.SubElement(parent_body, "body")
        follower_body.set("name", follower_body_name)
        follower_body.set("pos", fmt(FOLLOWER_PARENT_POS_MM[parent_name]))

        joint = ET.SubElement(follower_body, "joint")
        joint.set("name", f"{mech_name}_slot_slide")
        joint.set("type", "slide")
        joint.set("axis", AXIS)
        joint.set("range", RANGE)

        inertial = ET.SubElement(follower_body, "inertial")
        inertial.set("pos", "0 0 0")
        inertial.set("mass", FOLLOWER_MASS)
        inertial.set("diaginertia", FOLLOWER_INERTIA)

        follower_site_name = f"{mech_name}_slot_follower_site"
        follower_site = ET.SubElement(follower_body, "site")
        follower_site.set("name", follower_site_name)
        follower_site.set("pos", "0 0 0")

        # 3) equality connect
        connect = ET.SubElement(equality, "connect")
        connect.set("site1", pin_site_name)
        connect.set("site2", follower_site_name)

        print(f"{mech_name}: crank={crank_name} parent={parent_name} 완료")

    ET.indent(tree, space="  ")
    tree.write(DRAFT_PATH, xml_declaration=False)
    print(f"\n저장: {DRAFT_PATH}")


if __name__ == "__main__":
    main()
