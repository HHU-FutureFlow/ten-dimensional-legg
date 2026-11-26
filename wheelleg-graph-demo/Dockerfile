FROM node:24-bookworm-slim AS build
WORKDIR /app
COPY package.json package-lock.json ./
RUN npm ci
COPY . .
RUN npm run build

FROM node:24-bookworm-slim
WORKDIR /app
ENV NODE_ENV=production GRAPH_HOST=0.0.0.0 GRAPH_PORT=4317 GRAPH_DATA_DIR=/app/data/v2
COPY package.json package-lock.json ./
RUN npm ci --omit=dev && mkdir -p /app/data/v2 && chown -R node:node /app/data
COPY --from=build /app/dist ./dist
COPY server ./server
COPY shared ./shared
COPY schema ./schema
COPY scripts/graph-cli.mjs ./scripts/graph-cli.mjs
COPY examples ./examples
USER node
EXPOSE 4317
CMD ["node", "server/index.ts"]
