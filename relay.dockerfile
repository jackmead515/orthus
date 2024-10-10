FROM node:20-slim

COPY ./relay /usr/src/app

WORKDIR /usr/src/app

RUN npm install

EXPOSE 5454

CMD [ "node", "src/index.js" ]