#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
RovPang 학습 데이터 수집 스크립트 (JetRover RGB 카메라)

rgb_viewer.py와 같은 방식으로 카메라를 열고, 사진을 저장한다.
화면이 있는 환경(NoMachine)과 화면이 없는 환경(PuTTY) 모두에서 동작한다.

사용 예)
  # PuTTY: 터미널에서 Enter를 누를 때마다 1장 저장, q + Enter 로 종료
  python3 capture_dataset.py --label box

  # PuTTY: 1초마다 자동 저장, 100장 찍으면 종료
  python3 capture_dataset.py --label sign_A --interval 1 --count 100

  # NoMachine: 미리보기 창에서 s = 저장, q = 종료
  python3 capture_dataset.py --label box --preview
"""
import argparse
import os
import select
import sys
import time

import cv2


def parse_args():
    p = argparse.ArgumentParser(description="RovPang dataset capture")
    p.add_argument("--label", required=True,
                   help="저장할 대상 이름 (예: box, sign_A, sign_B, background)")
    p.add_argument("--out", default=os.path.expanduser("~/rovpang_dataset"),
                   help="저장 폴더 (기본: ~/rovpang_dataset)")
    p.add_argument("--interval", type=float, default=0.0,
                   help="자동 저장 간격(초). 0이면 수동 저장")
    p.add_argument("--count", type=int, default=0,
                   help="이 장수만큼 저장하면 종료. 0이면 제한 없음")
    p.add_argument("--width", type=int, default=640)
    p.add_argument("--height", type=int, default=360)
    p.add_argument("--device", type=int, default=0, help="/dev/videoN 의 N")
    p.add_argument("--preview", action="store_true",
                   help="미리보기 창 띄우기 (NoMachine 등 화면이 있을 때만)")
    return p.parse_args()


def open_camera(args):
    cap = cv2.VideoCapture(args.device, cv2.CAP_V4L2)
    if not cap.isOpened():
        print("카메라 연결 실패! 다른 프로그램(rgb_viewer.py 등)이 카메라를 쓰고 있지 않은지 확인하세요.")
        raise SystemExit(1)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
    # 처음 몇 프레임은 노출이 안정되지 않으므로 버린다
    for _ in range(10):
        cap.read()
    w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    print("카메라 연결 성공! 해상도 {}x{}".format(w, h))
    return cap


def next_index(folder, label):
    """이미 저장된 파일 다음 번호부터 이어서 저장한다 (덮어쓰기 방지)."""
    nums = []
    for name in os.listdir(folder):
        if name.startswith(label + "_") and name.endswith(".jpg"):
            try:
                nums.append(int(name[len(label) + 1:].split("_")[0]))
            except ValueError:
                pass
    return max(nums) + 1 if nums else 0


def save(frame, folder, label, idx):
    # 로봇 시계가 맞지 않을 수 있으므로 번호를 먼저 두고, 시각은 참고용으로만 붙인다
    name = "{}_{:04d}_{}.jpg".format(label, idx, time.strftime("%H%M%S"))
    path = os.path.join(folder, name)
    cv2.imwrite(path, frame)
    print("저장: {}".format(path))


def stdin_line():
    """터미널 입력이 있으면 한 줄 읽고, 없으면 None (기다리지 않음)."""
    if select.select([sys.stdin], [], [], 0)[0]:
        return sys.stdin.readline().strip().lower()
    return None


def main():
    args = parse_args()
    folder = os.path.join(args.out, args.label)
    os.makedirs(folder, exist_ok=True)
    idx = next_index(folder, args.label)
    saved = 0

    if args.preview and not os.environ.get("DISPLAY"):
        print("화면(DISPLAY)이 없어 미리보기를 끕니다. PuTTY에서는 --preview 없이 실행하세요.")
        args.preview = False

    cap = open_camera(args)

    if args.interval > 0:
        print("{}초마다 자동 저장합니다. 멈추려면 q 입력 후 Enter (또는 Ctrl+C)".format(args.interval))
    elif args.preview:
        print("미리보기 창에서 s = 저장, q = 종료")
    else:
        print("Enter = 1장 저장, q + Enter = 종료")

    last = 0.0
    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                print("영상 수신 실패!")
                break

            want_save = False
            quit_now = False

            # 터미널 입력 (PuTTY)
            line = stdin_line()
            if line is not None:
                if line == "q":
                    quit_now = True
                elif args.interval <= 0:
                    want_save = True

            # 자동 저장
            now = time.time()
            if args.interval > 0 and now - last >= args.interval:
                want_save = True
                last = now

            # 미리보기 창 (NoMachine)
            if args.preview:
                view = frame.copy()
                cv2.putText(view, "{}  saved: {}".format(args.label, saved), (10, 25),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                cv2.imshow("RovPang - capture", view)
                key = cv2.waitKey(1) & 0xFF
                if key == ord("q"):
                    quit_now = True
                elif key == ord("s"):
                    want_save = True

            if want_save:
                save(frame, folder, args.label, idx)
                idx += 1
                saved += 1
                if args.count and saved >= args.count:
                    print("{}장 저장 완료".format(saved))
                    break

            if quit_now:
                break
    except KeyboardInterrupt:
        pass
    finally:
        cap.release()
        if args.preview:
            cv2.destroyAllWindows()
        print("이번에 저장한 사진: {}장 -> {}".format(saved, folder))


if __name__ == "__main__":
    main()
