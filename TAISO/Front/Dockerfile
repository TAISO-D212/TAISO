# Node.js 20.10.1 버전을 기반으로 한 이미지 사용
FROM node:20.11.1 AS build

# Enable BuildKit
ENV DOCKER_BUILDKIT=1

# 작업 디렉토리 설정
WORKDIR /app

# 소스 코드 복사
COPY . .

# npm install을 하기 전에 불필요한 파일을 삭제
RUN rm -rf /app/node_modules /app/package-lock.json

# 필요한 패키지 설치
RUN npm install

# 빌드된 파일을 호스트 시스템으로 복사
RUN npm run build
COPY dist /app/dist