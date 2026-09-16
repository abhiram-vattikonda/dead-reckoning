# Offline OSM road graph

Place a converted graph at `app/src/main/assets/road_graph.json` before building.
The JSON format is produced by the companion converter:

```bash
PYTHONPATH=src:. python scripts/osm_to_roadgraph.py region.osm road_graph.json
cp road_graph.json /path/to/dead\ Receoning/app/src/main/assets/road_graph.json
```

The graph contains an ENU `origin`, `nodes` in metres, and `edges` with optional
`oneway`. The Android app snaps the current GPS start and destination to the
nearest graph nodes and runs Dijkstra, then feeds the ordered road polyline to
the route-constrained fusion engine. Without this asset, the app explicitly
falls back to a straight-line route.
