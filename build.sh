docker build -f app.dockerfile -t orthus-app .

docker build -f relay.dockerfile -t orthus-relay .

docker tag orthus-app:latest jackmead515/orthus-app:$1

docker tag orthus-relay:latest jackmead515/orthus-relay:$1

docker push jackmead515/orthus-app:$1

docker push jackmead515/orthus-relay:$1
