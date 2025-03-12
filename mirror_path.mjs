#!/usr/bin/env zx
const pathFile = argv._[0];
const pathContents = await fs.readFile(pathFile, "utf8");
const path = JSON.parse(pathContents);

const FIELD_CENTER_OFFSET = 8.052 / 2;

function mirror(point) {
  return (point - FIELD_CENTER_OFFSET) * -1 + FIELD_CENTER_OFFSET;
}

for (const waypoint of path.waypoints) {
  waypoint.anchor.y = mirror(waypoint.anchor.y);
  if (waypoint.nextControl) {
    waypoint.nextControl.y = mirror(waypoint.nextControl.y);
  }
  if (waypoint.prevControl) {
    waypoint.prevControl.y = mirror(waypoint.prevControl.y);
  }
}

path.goalEndState.rotation *= -1;
path.idealStartingState.rotation *= -1;

console.log(JSON.stringify(path, null, 4));
