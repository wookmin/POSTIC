"""카메라 캡처. 열기 실패와 프레임 유실을 여기서 흡수한다."""

import platform

import cv2

# 플랫폼별 캡처 백엔드. Linux 는 V4L2 가 안정적이고,
# macOS 는 AVFoundation, Windows 는 DSHOW 를 쓴다.
_BACKENDS = {
    "Linux": cv2.CAP_V4L2,
    "Darwin": cv2.CAP_AVFOUNDATION,
    "Windows": cv2.CAP_DSHOW,
}
_DEFAULT_BACKEND = _BACKENDS.get(platform.system(), cv2.CAP_ANY)


class CameraError(RuntimeError):
    pass


class CameraStream:
    """source 는 인덱스(0) 또는 장치 경로("/dev/video2") 둘 다 받는다.

    USB 포트나 부팅 순서에 따라 인덱스는 흔들리지만 경로는 안정적이므로,
    장치가 여러 개일 때는 경로로 지정하는 편이 안전하다.
    """

    def __init__(self, source=0, width=640, height=480):
        self.source = source
        self.width = width
        self.height = height
        self._capture = None

    def __enter__(self):
        source = self.source
        if isinstance(source, str) and source.isdigit():
            source = int(source)
        self._capture = cv2.VideoCapture(source, _DEFAULT_BACKEND)
        if not self._capture.isOpened():
            self._capture.release()
            raise CameraError(
                f"카메라 {self.source} 를 열 수 없습니다. "
                "다른 프로그램이 쓰고 있는지, 장치가 있는지 확인하세요.")
        self._capture.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        self._capture.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        return self

    def __exit__(self, *exc):
        if self._capture is not None:
            self._capture.release()
        self._capture = None
        return False

    def read(self):
        """BGR 프레임. 읽지 못하면 None."""
        if self._capture is None:
            raise CameraError("카메라가 열려 있지 않습니다")
        ok, frame = self._capture.read()
        if not ok or frame is None:
            return None
        # 거울처럼 보이는 편이 자세를 확인하기 쉽다.
        return cv2.flip(frame, 1)
