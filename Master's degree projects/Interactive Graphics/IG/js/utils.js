function resizeRendererToDisplaySize(renderer) {
  const canvas = renderer.domElement;
  const width = canvas.clientWidth;
  const height = canvas.clientHeight;
  const needResize = canvas.width !== width || canvas.height !== height;
  if (needResize) {
    renderer.setSize(width, height, false);
  }
  return needResize;
}

// RANDOM VALUE IN RANGE [min, max)
function randomValue(min, max) {
  return Math.random() * (max - min) + min;
}

function getRandomInt(min, max) {
  min = Math.ceil(min);
  max = Math.floor(max);
  return Math.floor(Math.random() * (max+1 - min) + min);
  // The maximum is inclusive and the minimum is inclusive
}

function getRandomRoadLocations(nPairs, maxRoadNumbers) {
  var roads = [];
  var deg = 0.0;
  for (var i = 0; i < nPairs; i++) {
    var exist, nr;
    // first segment always without roads to position the character each segment following a segment with 3 roads must be empty
    if (i == 0) exist = 0;
    if (i != 0 && roads[i-1][2] != 3) exist = getRandomInt(0, 1);
    if (i != 0 && roads[i-1][2] == 3) exist = 0;

    if (exist == 0) nr = 0;
    if (exist == 1) nr = getRandomInt(1, maxRoadNumbers);

    roads.push([deg, exist, nr]);

    deg += 7.5; // 360 / 7.5 = 48 segments ==> nPairs = 48 !
  }

  return roads;
}

function squared_distance(herox, heroz, carx, carz) {
  return Math.sqrt(Math.pow(herox-carx, 2) + Math.pow(heroz-carz));
}

let done = false;

function move_camera_up(controls, y) {
  controls.target.set(0, y, 0);
  controls.update();
  if (y > 125 && !done) {
    sessionStorage.setItem('passaggioScore', document.getElementById("score").innerHTML);
    done = true;
    location.href = './gameover.html';
  }
}

function animate_coins(coinsCollisions) {
  for (let coin of coinsCollisions) {
    coin.rotation.y += 0.1;
    if (coin.rotation.y == 360) coin.rotation.y = 0.1;
  }
}