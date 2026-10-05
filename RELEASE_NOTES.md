# Dataset v1 Release Notes

## 포함 파일

- `rearend-v1-part1.zip`: 후방추돌 영상 162개
- `rearend-v1-part2.zip`: 후방추돌 영상 162개
- `intersection-v1.zip`: 교차로 충돌 영상 324개
- `lanechange-v1.zip`: 동시 차선변경 영상 324개
- `metadata-v1.zip`: 통합 Excel 메타데이터, 스키마와 README
- `SHA256SUMS.txt`: 배포 파일 무결성 확인용 체크섬

## 데이터 요약

- 전체 영상: 972개
- 충돌 영상: 444개
- 비충돌 영상: 528개
- 전체 용량: 5.18 GB

## 압축 해제

각 ZIP 파일은 독립적으로 압축을 해제할 수 있습니다. 후방추돌 데이터만 두 파일로 나뉘며 두 파일의 내용을 동일한 폴더에 풀면 324개 영상이 구성됩니다.

## 체크섬 확인

Windows PowerShell에서는 다음 명령으로 다운로드 파일의 SHA-256 값을 확인할 수 있습니다.

```powershell
Get-FileHash .\rearend-v1-part1.zip -Algorithm SHA256
```

출력값을 `SHA256SUMS.txt`와 비교합니다.
