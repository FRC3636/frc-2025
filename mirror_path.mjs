#!/usr/bin/env zx

// Mirror all paths in the pathplanner/paths directory with "Blue" in the name,
// re-saving them as a "Red" variant.
//
// Usage: zx mirror_path.mjs [path]
//
// > npm install zx
// https://google.github.io/zx

const pathFile = argv._[0];

const FIELD_CENTER_OFFSET = 8.052 / 2;

function mirrorPoint(point) {
  return (point - FIELD_CENTER_OFFSET) * -1 + FIELD_CENTER_OFFSET;
}

async function mirrorPath(fileName) {
  const path = await fs.readJson(fileName);

  for (const waypoint of path.waypoints) {
    waypoint.anchor.y = mirrorPoint(waypoint.anchor.y);
    if (waypoint.nextControl) {
      waypoint.nextControl.y = mirrorPoint(waypoint.nextControl.y);
    }
    if (waypoint.prevControl) {
      waypoint.prevControl.y = mirrorPoint(waypoint.prevControl.y);
    }
  }

  path.goalEndState.rotation *= -1;
  path.idealStartingState.rotation *= -1;

  return JSON.stringify(path, null, 4);
}

if (pathFile) {
  console.log(await mirrorPath(pathFile));
} else {
  const prefix = "src/main/deploy/pathplanner/paths";

  const files = await fs.readdir(prefix);
  for (const file of files) {
    if (file.endsWith(".path") && file.includes("Blue")) {
      const newName = file.replace("Blue", "Red");
      const mirrored = await mirrorPath(`${prefix}/${file}`);
      console.log(newName, mirrored);
      await fs.writeFile(`${prefix}/${newName}`, mirrored);
    }
  }
}
