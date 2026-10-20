#!/usr/bin/env bash
# =====================================================================
# SCV 운영 DB 구성 (3차) — 관제 PC(Victus)에서 실행
#   1) PostgreSQL 16 컨테이너 robotdb4_postgresql  (영구 데이터, 볼륨 robotdata4_postgresql)
#   2) 스키마(01) · 초기값(02) 적용, 규칙 검증(03)은 --verify 일 때만
#   실시간 상태(로그인 세션 · 로봇 최신 상태)는 3-2차부터 관제 PC server_page 메모리에 둠 — Redis 컨테이너 없음
#
#   사용:  ./setup_db.sh                   # 컨테이너 생성 + 스키마·초기값 적용 (운영 DB robotdb4_postgresql)
#          ./setup_db.sh --verify          # 위 + 03_verify.sql (모두 ROLLBACK, 데이터 남지 않음)
#          ./setup_db.sh --test            # 같은 컨테이너에 시험 DB robotdb4_test를 만들어 적용 (운영 DB는 그대로)
#          ./setup_db.sh --test --verify   # 시험 DB에 적용 + 검증
#          ./setup_db.sh --test --reset    # 시험 DB를 지우고 처음부터 다시 만듦 (운영 DB는 건드리지 않음)
#   여러 번 실행해도 됨: 컨테이너·볼륨이 있으면 그대로 쓰고, SQL은 IF NOT EXISTS / ON CONFLICT
# =====================================================================
set -euo pipefail

HERE="$(cd "$(dirname "$0")" && pwd)"
SQL_DIR="$HERE/../sql"

PG_NAME=robotdb4_postgresql
PG_VOLUME=robotdata4_postgresql
PG_DB=robotdb4_postgresql
PG_USER=rokey
PG_PASSWORD=rokey

TEST_DB=robotdb4_test

VERIFY=0; TEST=0; RESET=0
for a in "$@"; do
    case "$a" in
        --verify) VERIFY=1 ;;
        --test)   TEST=1 ;;
        --reset)  RESET=1 ;;
        *) echo "알 수 없는 옵션: $a (--verify · --test · --reset)"; exit 1 ;;
    esac
done
if [[ $RESET -eq 1 && $TEST -eq 0 ]]; then
    echo "--reset은 --test와 함께만 씁니다 (운영 DB는 지우지 않음)"; exit 1
fi
TARGET_DB=$PG_DB
[[ $TEST -eq 1 ]] && TARGET_DB=$TEST_DB

exists() { docker ps -a --format '{{.Names}}' | grep -qx "$1"; }
running() { docker ps --format '{{.Names}}' | grep -qx "$1"; }

# ---------------------------------------------------------------------
# 1. PostgreSQL 16
# ---------------------------------------------------------------------
echo "[1/3] PostgreSQL 컨테이너"
docker pull postgres:16
docker volume create "$PG_VOLUME" >/dev/null
if ! exists "$PG_NAME"; then
    docker run -d --name "$PG_NAME" \
        -e POSTGRES_DB="$PG_DB" \
        -e POSTGRES_USER="$PG_USER" \
        -e POSTGRES_PASSWORD="$PG_PASSWORD" \
        -p 5432:5432 \
        -v "$PG_VOLUME":/var/lib/postgresql/data \
        postgres:16
elif ! running "$PG_NAME"; then
    docker start "$PG_NAME" >/dev/null
fi

# 첫 실행 때 이미지가 임시 서버로 초기화한 뒤 다시 시작하므로, TCP 접속이 될 때까지 대기
echo -n "      접속 대기"
for _ in $(seq 1 60); do
    if docker exec "$PG_NAME" pg_isready -h 127.0.0.1 -U "$PG_USER" -d "$PG_DB" >/dev/null 2>&1; then
        echo " → 준비됨"; break
    fi
    echo -n "."; sleep 1
done
docker exec "$PG_NAME" pg_isready -h 127.0.0.1 -U "$PG_USER" -d "$PG_DB" >/dev/null

psql_run() {
    docker exec -i "$PG_NAME" psql -h 127.0.0.1 -U "$PG_USER" -d "$TARGET_DB" -v ON_ERROR_STOP=1 -q "$@"
}
psql_admin() {     # 관리용 — postgres DB에 접속 (DB 만들기·지우기)
    docker exec -i "$PG_NAME" psql -h 127.0.0.1 -U "$PG_USER" -d postgres -v ON_ERROR_STOP=1 -q "$@"
}

# 시험 DB — 같은 컨테이너·같은 계정, 운영 DB와 데이터가 섞이지 않음
if [[ $TEST -eq 1 ]]; then
    if [[ $RESET -eq 1 ]]; then
        echo "      시험 DB $TEST_DB 지우기"
        psql_admin -c "DROP DATABASE IF EXISTS $TEST_DB WITH (FORCE)"
    fi
    if ! psql_admin -Atc "SELECT 1 FROM pg_database WHERE datname = '$TEST_DB'" | grep -q 1; then
        echo "      시험 DB $TEST_DB 만들기"
        psql_admin -c "CREATE DATABASE $TEST_DB"
    fi
fi

# ---------------------------------------------------------------------
# 2. 스키마 · 초기값
# ---------------------------------------------------------------------
echo "[2/3] 스키마 적용 (01_schema.sql) → $TARGET_DB"
psql_run < "$SQL_DIR/01_schema.sql"
echo "      초기값 적용 (02_seed.sql)"
psql_run < "$SQL_DIR/02_seed.sql"

if [[ $VERIFY -eq 1 ]]; then
    echo "[3/3] 규칙 검증 (03_verify.sql)"
    psql_run < "$SQL_DIR/03_verify.sql"
else
    echo "[3/3] 규칙 검증 건너뜀 (--verify로 실행)"
fi

# ---------------------------------------------------------------------
echo
echo "완료"
docker exec "$PG_NAME" psql -h 127.0.0.1 -U "$PG_USER" -d "$TARGET_DB" -Atc \
    "SELECT '  PostgreSQL 테이블 ' || count(*) || '개 ($TARGET_DB)' FROM information_schema.tables WHERE table_schema = 'public'"
echo "  PostgreSQL  postgresql://$PG_USER:****@<관제PC>:5432/$TARGET_DB"
