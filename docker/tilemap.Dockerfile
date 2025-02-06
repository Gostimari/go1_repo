# tilemap.dockerfile
FROM overv/openstreetmap-tile-server:latest

RUN sed -i 's/\luxembourg\>/portugal/g' run.sh

ENV THREADS 24

RUN ./run.sh import