# CARLA Fixed CCTV Accident Data

시뮬레이션 기반 고정형 CCTV 관점의 교통사고 영상 및 메타데이터 생성 연구를 소개하는 GitHub Pages 사이트입니다.

## 현재 구성

- `index.html`: 프로젝트 소개 페이지
- `styles.css`: 반응형 화면 스타일
- `assets/`: CARLA 장면과 데이터 생성 절차 이미지
- `metadata-schema.csv`: 공개 메타데이터 구조
- `tests/`: 시뮬레이션과 영상 메타데이터 단위 테스트

## 연구 범위

- CARLA Town04 도로 환경
- 후방추돌, 교차로 충돌 및 동시 차선변경 사고 시나리오
- 지면 기준 9 m 높이의 고정형 RGB 카메라
- 1280×720, 60 FPS MP4 영상
- 차량 조건, 충돌 프레임과 영상 정보를 포함하는 Excel 메타데이터

## 데이터셋 현황

| 구분 | 영상 수 | 용량 |
|---|---:|---:|
| RearEnd | 324 | 2.25 GB |
| Intersection | 324 | 1.31 GB |
| LaneChange | 324 | 1.62 GB |
| 합계 | 972 | 5.18 GB |

- 충돌 영상: 444개
- 비충돌 영상: 528개
- 통합 메타데이터: 972행

영상 파일은 일반 Git 커밋에 포함하지 않습니다. GitHub Release의 2 GB 미만 분할 파일로 배포하고, 웹페이지와 코드만 저장소에서 관리합니다.

자세한 데이터 구성과 활용 범위는 [DATASET_CARD.md](DATASET_CARD.md)를 참고합니다.

## Release 배포 파일

| 파일 | 포함 영상 | 크기 |
|---|---:|---:|
| `rearend-v1-part1.zip` | 162 | 1.19 GB |
| `rearend-v1-part2.zip` | 162 | 1.06 GB |
| `intersection-v1.zip` | 324 | 1.31 GB |
| `lanechange-v1.zip` | 324 | 1.62 GB |
| `metadata-v1.zip` | Excel 및 문서 3개 | 0.1 MB 미만 |

각 파일의 SHA-256 값은 Release에 함께 올리는 `SHA256SUMS.txt`에서 확인할 수 있습니다.

## 로컬 확인

`index.html`을 웹 브라우저에서 열면 별도 빌드 과정 없이 확인할 수 있습니다.

## GitHub Pages 게시

1. GitHub에서 `carla-cctv-accident-dataset` 이름의 공개 저장소를 만듭니다.
2. 이 폴더의 파일을 저장소 기본 브랜치에 올립니다.
3. 저장소의 `Settings`에서 `Pages`로 이동합니다.
4. `Build and deployment`의 Source를 `Deploy from a branch`로 설정합니다.
5. 기본 브랜치와 `/(root)` 폴더를 선택한 뒤 저장합니다.

게시 후 주소는 일반적으로 다음 형식입니다.

```text
https://milkyssoda.github.io/carla-cctv-accident-dataset/
```

GitHub 공식 문서: [Creating a GitHub Pages site](https://docs.github.com/en/pages/getting-started-with-github-pages/creating-a-github-pages-site), [Configuring a publishing source](https://docs.github.com/en/pages/getting-started-with-github-pages/configuring-a-publishing-source-for-your-github-pages-site)

## 공개 전 확인 항목

- 생성 영상과 메타데이터의 최종 수량
- 코드와 데이터의 라이선스
- 논문 제목, 저자, 소속과 인용 형식
- 대용량 영상 파일의 배포 위치

## 공개 상태

영상 972개와 메타데이터 972행을 포함한 [Dataset v1.0.0](https://github.com/MilkysSoda/carla-cctv-accident-dataset/releases/tag/v1.0.0)을 공개했습니다. 생성 코드와 테스트도 저장소에서 제공합니다.
