FROM overv/openstreetmap-tile-server:1.3.10

RUN apt install -y wget

RUN wget 'https://download.geofabrik.de/europe/portugal-latest.osm.pbf'

RUN mv portugal-latest.osm.pbf /data.osm.pbf

ENV THREADS 24

ENV ALLOW_CORS true

RUN ./run.sh import