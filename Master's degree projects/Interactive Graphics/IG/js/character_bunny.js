import * as THREE from 'https://threejs.org/build/three.module.js';
import {GLTFLoader} from 'https://threejs.org/examples/jsm/loaders/GLTFLoader.js';

import * as MODELS from './models.js';
import * as OBJECTS from './objects.js'
import * as GAME from './game.js'

let body, hip, spine, legL, legR, armL, armR, EarL, EarR;
let go_left, go_right, go_up, checkAnimating;
let up_dir = true;
let move = true;
export let hero_position = {x: 0, y: 100.93, z: 0};
export let bunny_score = 0;
export let coin_collision = false;

// function needed to represent the hierarchical structure of the hero/main character
function dumpObject(obj, lines = [], isLast = true, prefix = '') {
  const localPrefix = isLast ? '└─' : '├─';
  lines.push(`${prefix}${prefix ? localPrefix : ''}${obj.name || '*no-name*'} [${obj.type}]`);
  const newPrefix = prefix + (isLast ? '  ' : '│ ');
  const lastNdx = obj.children.length - 1;
  obj.children.forEach((child, ndx) => {
    const isLast = ndx === lastNdx;
    dumpObject(child, lines, isLast, newPrefix);
  });
  return lines;
}

function create_bunny(scene, night) {
    // Allows to decide the influence of light on the bunny during the day and at night
    let luceBunny;
    if(night) luceBunny = 0;
    else luceBunny = 0.4;

    const gltfLoader = new GLTFLoader(MODELS.loadingManager);
    //to apply texture
    const textureLoader = new THREE.TextureLoader(MODELS.loadingManager);
    const texture = textureLoader.load('assets/models/level1/bunny/rabbit_final_tex.png')
    texture.flipY = false;
    gltfLoader.load('assets/models/level1/bunny/rabbit_final.gltf', (gltf) => {
      const root = gltf.scene;
      // to apply texture
      root.traverse ( ( o ) => {
        if (o.isMesh ) {
          o.material = new THREE.MeshStandardMaterial({map: texture});
          o.material.skinning = true;
          o.receiveShadow = true
          o.castShadow = true;
          o.material.metalness = luceBunny;
        }
      });

      const r = 100;
      const theta = 0;
      const y = r*Math.cos(theta) + 0.93;
      const z = r*Math.sin(theta);

      root.scale.set(0.6, 0.6, 0.6);
      root.position.set(0, y, z);
      root.rotation.y = Math.PI;
      root.name = "Bunny";

      // Print the hierarchical model of the main character 
      //console.log(dumpObject(root).join('\n'));

      scene.add(root);

      //PER CONIGLIO MICHELA
      body = root.getObjectByName('Armature');
      hip = body.children[0];
      legL = hip.children[1];
      legR = hip.children[2];
      spine = hip.children[0];
      armL = spine.children[1];
      armR = spine.children[2];
      EarL = root.getObjectByName('EarL');
      EarR = root.getObjectByName('EarR');

    });

}

function animate(scene, hModel){

  let mov;
  let initial_pos;   //initial position of the body of the rabbit
  let frontal_shift;  //how much the rabbit moves frontally
  let shift_relative_to_world;  // position in the world cartesian system
  let lat_shift_relative_to_world;
  let dx_lateral_shift;
  let sx_lateral_shift;

  if(!checkAnimating){
    if(go_up) {
      bunny_score += 1;
      document.getElementById("score").innerHTML = "score: " + bunny_score;
      checkAnimating = true;
      initial_pos = body.position.y; //z;
      frontal_shift = 5;
      body.rotation.y = 0;
      shift_relative_to_world = initial_pos + frontal_shift;
      check_collisions_obstacles_front();
      if (move)
        mov = setInterval(function(){move_up(frontal_shift, shift_relative_to_world, mov, scene, hModel);}, 28);
    }
    else if (go_right) {
      checkAnimating = true;
      body.rotation.y = 80;
      initial_pos = body.position.x;
      dx_lateral_shift = 5;
      lat_shift_relative_to_world = initial_pos - dx_lateral_shift;
      check_collisions_obstacles_around(1);
      if (move)
        mov = setInterval(function(){move_right(dx_lateral_shift, lat_shift_relative_to_world, mov); }, 28);
        hero_position.x += 3;
    }
    else if (go_left) {
      checkAnimating = true;
      body.rotation.y = -80;
      initial_pos = body.position.x;
      sx_lateral_shift = 5;
      lat_shift_relative_to_world = initial_pos + sx_lateral_shift;
      check_collisions_obstacles_around(-1);
      if (move)
        mov = setInterval(function(){move_left(sx_lateral_shift, lat_shift_relative_to_world, mov); }, 28);
        hero_position.x -= 3;
    }
  }

}


function move_right(dx_lateral_shift, lat_shift_relative_to_world, mov){

  body.position.x -= 0.5;

  let s = dx_lateral_shift/2;

  if(body.position.x >= lat_shift_relative_to_world+s){
    body.rotation.y += 0.1;
    body.position.y += 0.5;

    armR.rotation.x += 0.15;  // arms move forward
    armL.rotation.x += 0.15;
    legR.rotation.x += 0.15;  // legs move backword
    legL.rotation.x += 0.15;
    EarL.rotation.x -= 0.22;
    EarR.rotation.x -= 0.22;

  } else if(body.position.x < lat_shift_relative_to_world+s && body.position.x >= lat_shift_relative_to_world){
    body.rotation.y -= 0.1;
    body.position.y -= 0.5;

    armR.rotation.x -= 0.15;  // arms move forward
    armL.rotation.x -= 0.15;
    legR.rotation.x -= 0.15;  // legs move backword
    legL.rotation.x -= 0.15;
    EarL.rotation.x += 0.22;
    EarR.rotation.x += 0.22;

    if(body.position.x == lat_shift_relative_to_world){
      go_right = false;
      clearInterval(mov);
      checkAnimating = false;
    }
  }

}

function move_left(sx_lateral_shift, lat_shift_relative_to_world, mov){

  body.position.x += 0.5;

  let s = sx_lateral_shift/2;

  if(body.position.x <= lat_shift_relative_to_world-s){
    body.rotation.y -= 0.1;
    body.position.y += 0.5;

    armR.rotation.x += 0.15;  // arms move forward
    armL.rotation.x += 0.15;
    legR.rotation.x += 0.15;  // legs move backword
    legL.rotation.x += 0.15;
    EarL.rotation.x -= 0.22;
    EarR.rotation.x -= 0.22;

  } else if(body.position.x > lat_shift_relative_to_world-s && body.position.x <= lat_shift_relative_to_world){
    body.rotation.y += 0.1;
    body.position.y -= 0.5;

    armR.rotation.x -= 0.15;  // arms move forward
    armL.rotation.x -= 0.15;
    legR.rotation.x -= 0.15;  // legs move backword
    legL.rotation.x -= 0.15;
    EarL.rotation.x += 0.22;
    EarR.rotation.x += 0.22;

    if(body.position.x == lat_shift_relative_to_world){
      go_left = false;
      clearInterval(mov);
      checkAnimating = false;
    }
  }

}




function move_up(frontal_shift, shift_relative_to_world, mov, scene, hModel){

  let x = frontal_shift/2;

  hModel.rotation.x += 0.00365;

  // the bunny goes up
  if(body.position.y < shift_relative_to_world-x && up_dir==true){
    body.position.y += 0.5;
    body.rotation.x -= 0.05;

    armR.rotation.x += 0.15;  // arms move forward
    armL.rotation.x += 0.15;
    legR.rotation.x += 0.15;  // legs move backword
    legL.rotation.x += 0.15;
    EarL.rotation.x -= 0.20;
    EarR.rotation.x -= 0.20;

    // keep moving up
    up_dir = true;
  } else if (body.position.y >= shift_relative_to_world-x && body.position.y < shift_relative_to_world && up_dir==true){
    body.position.y += 0.5;
    body.rotation.x -= 0.05;

    armR.rotation.x += 0.15;  // arms move forward
    armL.rotation.x += 0.15;
    legR.rotation.x += 0.15;  // legs move backword
    legL.rotation.x += 0.15;
    EarL.rotation.x -= 0.20;
    EarR.rotation.x -= 0.20;

    // now the bunny has to return to the ground moving down
    up_dir = false;

  // the bunny goes down
  } else if (body.position.y >= shift_relative_to_world-x && body.position.y < shift_relative_to_world && up_dir==false) {
    body.position.y -= 0.5;
    body.rotation.x += 0.05;

    armR.rotation.x -= 0.15;  // arms move forward
    armL.rotation.x -= 0.15;
    legR.rotation.x -= 0.15;  // legs move backword
    legL.rotation.x -= 0.15;
    EarL.rotation.x += 0.20;
    EarR.rotation.x += 0.20;

    // keep moving down
    up_dir = false;

  } else if (body.position.y < shift_relative_to_world-x  && up_dir==false) {
    body.position.y -= 0.5;
    body.rotation.x += 0.05;

    armR.rotation.x -= 0.15;   // arms move forward
    armL.rotation.x -= 0.15;
    legR.rotation.x -= 0.15;   // legs move beckword
    legL.rotation.x -= 0.15;
    EarL.rotation.x += 0.20;
    EarR.rotation.x += 0.20;

    if(body.position.y == 0) {
      up_dir = true;
      go_up = false;
      clearInterval(mov);  // necessary to stop the setInterval call
      checkAnimating = false;
    }
  }
}

function check_collisions_obstacles_front() {
  move = true;
  coin_collision = false;

  let pos = new THREE.Vector3();

  MODELS.bigObstaclesCollision.forEach(function (element, index) {
    let bigObs = MODELS.bigObstaclesCollision[index];
    let distx;
    let distz = 5;

    if (bigObs.name == 'Building1_Large') distx = 6.5;
    else if (bigObs.name == 'House1') distx = 3;
    else if (bigObs.name == 'Building2_Large') distx = 5.5;
    else if (bigObs.name == 'Building3_Big') distx = 5;
    else if (bigObs.name == 'Building4') distx = 4.8;
    else if (bigObs.name == 'Building3_Small') distx = 3;
    else distx = 4;

    pos.setFromMatrixPosition(bigObs.matrixWorld);
    if(pos.z >= -14 && pos.z < -1 && pos.y > 0 && hero_position.x <= pos.x + distx && hero_position.x >= pos.x - distx){ // check just for cars that are in the next slice, up to 7.5 deg (hero.position.z = 0 always)
        let distance = Math.sqrt(Math.pow(pos.z, 2));
        if(distance <= distz){
            move = false;
        }
    }
  });

  MODELS.smallObstaclesCollision.forEach(function (element, index) {
    let smallObs = MODELS.smallObstaclesCollision[index];
    pos.setFromMatrixPosition(smallObs.matrixWorld);

    let distx = 2.5;
    if (smallObs.name == 'BushBerries_1') distx = 2;
    else if (smallObs.name == 'BirchTree_1') distx = 1.5;
    if(pos.z >= -14 && pos.z < -2.5 && pos.y > 0 && hero_position.x <= pos.x + distx && hero_position.x >= pos.x - distx){ // check just for cars that are in the next slice, up to 7.5 deg (hero.position.z = 0 always)
        let distance = Math.sqrt(Math.pow(pos.z, 2));
        if(distance <= 5){
            move = false;
        }
    }
  });

  MODELS.coinsCollisions.forEach(function (element, index) {
    let coin = MODELS.coinsCollisions[index];
    pos.setFromMatrixPosition(coin.matrixWorld);

    let distx = 1;

    if(pos.z >= -5 && pos.z < 0 && pos.y > 0 && hero_position.x <= pos.x + distx && hero_position.x >= pos.x - distx){ // check just for cars that are in the next slice, up to 7.5 deg (hero.position.z = 0 always)
        let distance = Math.sqrt(Math.pow(pos.z, 2));
        if(coin.visible && distance <= 5){
          GAME.sound_audio4.play();
          coin.visible = false;
          coin_collision = true;
          bunny_score += 10;
          document.getElementById("score").innerHTML = "score: " + bunny_score;
        }
    }
  });
}

function check_collisions_obstacles_around(direction) {
  move = true;
  coin_collision = false;

  let pos = new THREE.Vector3();
  let dis;

  MODELS.bigObstaclesCollision.forEach(function (element, index) {
    let bigObs = MODELS.bigObstaclesCollision[index];
    pos.setFromMatrixPosition(bigObs.matrixWorld);

    if (bigObs.name == 'Building1_Large') dis = 10;
    else if (bigObs.name == 'Building2_Large') dis = 9;
    else if (bigObs.name == 'Building3_Big' || bigObs.name == 'Building4') dis = 7.5;
    else if (bigObs.name == 'Building1_Small' || bigObs.name == 'Building2_Small') dis = 7;
    else if (bigObs.name == 'Building3_Small') dis = 6.6;
    else if (bigObs.name == 'House1') dis = 6;
    else dis = 4.5;

    let distz = 4;
    if (bigObs.name == 'Building2_Small') distz = 3.5;

    if(pos.z >= -distz && pos.z < distz && pos.y > 0 && ((direction == -1 && pos.x <= hero_position.x) || (direction == 1 && pos.x >= hero_position.x))){ // check just for cars that are in the next slice, up to 7.5 deg (hero.position.z = 0 always)
        let distance = Math.sqrt(Math.pow(hero_position.x-pos.x, 2));
        if(distance <= dis){
            move = false;
        }
    }
  });

  MODELS.smallObstaclesCollision.forEach(function (element, index) {
    let smallObs = MODELS.smallObstaclesCollision[index];
    pos.setFromMatrixPosition(smallObs.matrixWorld);

    let distz = 2.5;
    if (smallObs.name == 'BirchTree_1') distz = 2;

    let distx = 5;
    if (smallObs.name == 'BirchTree_1') distx = 5.5;

    if(pos.z >= -distz && pos.z < distz && pos.y > 0 && ((direction == -1 && pos.x < hero_position.x) || (direction == 1 && pos.x > hero_position.x))){ // check just for cars that are in the next slice, up to 7.5 deg (hero.position.z = 0 always)
      let distance = Math.sqrt(Math.pow(hero_position.x-pos.x, 2));
      if(distance <= distx){
        move = false;

      }
    }
  });

  MODELS.coinsCollisions.forEach(function (element, index) {
    let coin = MODELS.coinsCollisions[index];
    pos.setFromMatrixPosition(coin.matrixWorld);

    let distz = 1.5;
    let distx = 4;

    if(pos.z >= -distz && pos.z < distz && pos.y > 0 && ((direction == -1 && pos.x < hero_position.x) || (direction == 1 && pos.x > hero_position.x))){ // check just for cars that are in the next slice, up to 7.5 deg (hero.position.z = 0 always)
      let distance = Math.sqrt(Math.pow(hero_position.x-pos.x, 2));
      if(coin.visible && distance <= distx){
          coin.visible = false;
          coin_collision = true;
          GAME.sound_audio4.play();
          bunny_score += 10;
          document.getElementById("score").innerHTML = "score: " + bunny_score;
      }
    }
  });
}



export function initialize_bunny(scene, hModel, night){
    create_bunny(scene, night); // body, hip, spine, legL, legR, armL, armR

    //// EVENT LISTENER MAIN CHARACTER ////
    go_left=false;
    go_right=false;
    go_up=false;
    checkAnimating=false;
    window.addEventListener("keydown", letsMove, false);

    function letsMove(event) {
      if(GAME.permessoPerMuoversi){
        if((event.keyCode === 37|| event.keyCode === 65) && body.position.x < 37) {  // arrow left or 'a' or 'A'
          check_collisions_obstacles_around(-1);
          if(!checkAnimating && move){
            go_left = true;
            GAME.sound_audio3.play();
          }
        }
        else if(event.keyCode === 38 || event.keyCode === 87) { // arrow up or 'w' or 'W'
          check_collisions_obstacles_front();
          if(!checkAnimating && move){
            go_up = true;
            GAME.sound_audio3.play();
          }
        }
        else if ((event.keyCode === 39 || event.keyCode === 68) && body.position.x > -37) { // arrow right or 'd' or 'D'
          check_collisions_obstacles_around(1);
          if(!checkAnimating && move){
            go_right = true;
            GAME.sound_audio3.play();
          }
        }
        animate(scene, hModel);
      }
    }
}
