#!/bin/bash

docker exec -it $(docker ps -a --filter "ancestor=xvisio-cam-dev" --format "{{.ID}}") /bin/bash