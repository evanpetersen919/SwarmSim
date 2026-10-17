FROM ubuntu:24.04 AS build
RUN apt-get update \
 && apt-get install -y --no-install-recommends build-essential cmake ninja-build git ca-certificates \
 && rm -rf /var/lib/apt/lists/*
WORKDIR /src
COPY . .
RUN cmake --preset release \
 && cmake --build --preset release \
 && ctest --preset release

FROM ubuntu:24.04
COPY --from=build /src/build/release/apps/swarm_sim /usr/local/bin/swarm_sim
ENTRYPOINT ["swarm_sim"]
CMD ["--seed", "42"]
