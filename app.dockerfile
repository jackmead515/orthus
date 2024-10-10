FROM node:20-slim as build_web

COPY ./app /usr/src/app/app
COPY ./server /usr/src/app/server

WORKDIR /usr/src/app

RUN cd app && npm install && npm run build

RUN mkdir -p server/static \
    && cp -r app/build/* server/static \
    && cp -r server/static/static/* server/static \
    && rm -rf server/static/static \
    && rm -rf app

FROM python:3.10-slim

# install ffmpeg
RUN apt-get update && apt-get install -y ffmpeg procps --no-install-recommends && rm -rf /var/lib/apt/lists/*

COPY --from=build_web /usr/src/app/server /usr/src/app/

WORKDIR /usr/src/app

RUN pip install --upgrade pip && pip install pipenv

RUN pipenv lock && pipenv install --system --deploy

# update permissions on app folder
RUN chmod -R 777 /usr/src/app

RUN mkdir -p /data

EXPOSE 8000

CMD [ "python3", "main.py" ]