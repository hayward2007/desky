"""이 PC가 같은 공유기 안에서 어떤 주소로 보이는지 찾아낸다.

## 왜 이 모듈이 필요한가

서버는 원래부터 `0.0.0.0`에 바인딩한다 — "이 컴퓨터의 **모든** 랜카드에서 받겠다"는
뜻이라, 같은 공유기에 붙은 폰·태블릿도 이미 접속할 수 있는 상태다.

문제는 **접속할 주소를 사람이 모른다는 것**이었다. 시작 로그에는
`https://0.0.0.0:8000`이라고만 찍히는데 `0.0.0.0`은 "아무 주소"라는 뜻의 특수한
값이라 폰 주소창에 입력할 수 있는 주소가 아니고, `localhost`는 폰 입장에서
자기 자신을 가리키므로 역시 안 된다. 폰이 입력해야 하는 건
`https://192.168.0.12:8000` 같은 **이 PC의 공유기 내부 주소**다.

그래서 이 모듈이 그 주소를 찾아 시작할 때 그대로 출력한다. 코드가 바뀌는 게 아니라
"이미 열려 있는 문의 번지수를 알려주는" 일이다.

## 어느 주소를 고르는가

랜카드가 여러 개인 경우가 흔하다 — 유선과 무선을 같이 꽂았거나, VirtualBox·WSL·
VPN·도커가 가상 어댑터를 만들어 두었거나. 그중 **폰과 같은 공유기에 물린 것** 하나를
골라야 하는데, 이름만 보고는 알 수 없다.

그래서 "바깥으로 나가려면 어느 랜카드를 쓰겠는가"를 운영체제에 물어본다
(`_primary_ipv4`). UDP 소켓을 공인 주소로 `connect()` 하면 실제로 패킷은 한 개도
나가지 않지만 커널이 라우팅 테이블을 보고 출발지 주소를 정해 주므로, 그 값을 읽으면
된다. 이게 보통 공유기에 물린 그 랜카드다.

다만 확실하지 않을 수 있으므로(예: VPN이 켜져 있으면 VPN 주소가 잡힌다) 나머지 사설
주소도 함께 찾아 목록으로 보여준다 — 첫 번째로 안 되면 다음 것을 시도할 수 있도록.
"""

import ipaddress
import socket

# 사설 주소 대역(공유기가 나눠주는 주소). 공인 주소나 링크로컬(169.254.x.x,
# DHCP를 못 받았을 때 붙는 주소)은 같은 공유기 접속용으로 쓸 수 없으므로 거른다.
_PRIVATE_NETWORKS = (
    ipaddress.ip_network("10.0.0.0/8"),
    ipaddress.ip_network("172.16.0.0/12"),
    ipaddress.ip_network("192.168.0.0/16"),
)


def _is_lan_ipv4(address: str) -> bool:
    """같은 공유기 안에서 쓸 수 있는 사설 IPv4인지."""
    try:
        ip = ipaddress.ip_address(address)
    except ValueError:
        return False
    return any(ip in network for network in _PRIVATE_NETWORKS)


def _primary_ipv4():
    """바깥으로 나갈 때 쓰는 랜카드의 주소. 못 찾으면 None.

    UDP 소켓을 만들어 공인 주소로 `connect()` 한다 — UDP는 연결이라는 개념이
    없어서 **패킷은 한 개도 나가지 않는다.** 커널이 라우팅 테이블을 보고 출발지
    주소만 정해 줄 뿐이라, 인터넷이 끊겨 있어도 동작한다. 랜카드가 여러 개일 때
    "공유기 쪽" 하나를 고르는 가장 확실한 방법이다.
    """
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        sock.connect(("8.8.8.8", 80))
        return sock.getsockname()[0]
    except OSError:
        return None
    finally:
        sock.close()


def _hostname_ipv4s():
    """호스트 이름으로 조회되는 모든 IPv4. 랜카드가 여러 개면 여럿 나온다."""
    try:
        infos = socket.getaddrinfo(socket.gethostname(), None, socket.AF_INET)
    except socket.gaierror:
        return []
    return [info[4][0] for info in infos]


def lan_addresses():
    """같은 공유기에서 이 PC로 접속할 수 있는 주소들. 가장 유력한 것이 맨 앞.

    비어 있으면 이 PC가 어느 네트워크에도 붙어 있지 않다는 뜻이다(랜선 빠짐,
    와이파이 미연결). 그 경우 폰에서 접속할 방법 자체가 없다.
    """
    ordered = []
    for address in [_primary_ipv4()] + _hostname_ipv4s():
        if address and _is_lan_ipv4(address) and address not in ordered:
            ordered.append(address)
    return ordered


def server_urls(port, scheme="https"):
    """폰 주소창에 입력할 URL 목록. 가장 유력한 것이 맨 앞."""
    return [f"{scheme}://{address}:{port}" for address in lan_addresses()]


def startup_banner(port, scheme="https"):
    """시작할 때 콘솔에 찍을 안내문을 만든다.

    로그 한 줄로 흘려보내지 않고 상자로 크게 그리는 이유: 시연 직전에 폰으로
    주소를 입력해야 하는 사람이 스크롤을 뒤지지 않고 바로 찾을 수 있어야 한다.
    접속이 안 될 때 확인할 것들도 여기 같이 적는다 — 실제로 막히는 원인은
    거의 항상 아래 셋 중 하나인데, 증상만 보면 서버 문제처럼 보이기 때문이다.
    """
    urls = server_urls(port, scheme)
    line = "─" * 62
    rows = [line, " 같은 공유기에 있는 기기에서 접속하세요", ""]

    if not urls:
        rows += [
            " ⚠ 이 PC가 네트워크에 붙어 있지 않습니다.",
            "   와이파이에 연결하거나 랜선을 꽂은 뒤 다시 실행하세요.",
        ]
    else:
        rows.append(f"   팔에 장착한 폰 →  {urls[0]}/mobile")
        rows.append(f"   PC 대시보드   →  {urls[0]}/")
        if len(urls) > 1:
            rows.append("")
            rows.append("   위 주소가 안 되면 (랜카드가 여러 개입니다):")
            for url in urls[1:]:
                rows.append(f"     {url}/mobile")

    rows += [
        "",
        " 접속이 안 되면",
        "   · 인증서 경고는 정상입니다 — 고급 → 계속으로 진행하세요",
        "     (카메라·마이크는 HTTPS에서만 열려서 자체 서명 인증서를 씁니다)",
        f"   · PC 방화벽에서 {port}번 포트로 들어오는 연결을 허용했는지",
        "   · 폰과 PC가 같은 와이파이인지 (5GHz/2.4GHz는 같아도 됩니다)",
        "   · 게스트 와이파이는 기기끼리 통신이 막혀 있습니다(AP 격리)",
        line,
    ]
    return "\n".join(rows)
