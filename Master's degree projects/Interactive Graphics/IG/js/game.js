import * as THREE from 'https://threejsfundamentals.org/threejs/resources/threejs/r127/build/three.module.js';
import {OrbitControls} from 'https://threejsfundamentals.org/threejs/resources/threejs/r127/examples/jsm/controls/OrbitControls.js';

import * as WORLD from './models.js'
import * as OBJECTS from './objects.js'
import * as CARS from './cars.js'

import * as CHARACTER_bunny from './character_bunny.js'
import * as CHARACTER_bear from './character_bear.js'
import * as CHARACTER_man from './character_man.js'

$(document).ready(function() {
  $("#back").click(function(){
      location.href='./main.html'
  });
  $("#restart").click(function(){
      location.reload()
  });

  sessionStorage.setItem('passaggioScore', 'score: 0');
});

export let night = false;
export let levelToPlay=1;
export let id;
export let sound_audio, sound_audio2, sound_audio3, sound_audio4;
export let controls, camera;
export let permessoPerMuoversi;
let y_target = 105;

export function changePermesso(){
  permessoPerMuoversi = true;
}

function main() {
  permessoPerMuoversi = false;
  // Reading Local Storage
  if(sessionStorage.getItem("dayOrNight")=="true") night=true;

  if(sessionStorage.getItem("levelPass")=="easy") levelToPlay=1;
  else if(sessionStorage.getItem("levelPass")=="medium") levelToPlay=2;
  else if(sessionStorage.getItem("levelPass")=="hard") levelToPlay=3;


  const pi = Math.PI;
  const canvas = document.querySelector('#c');
  const renderer = new THREE.WebGLRenderer({canvas});
  renderer.shadowMap.enabled = true;
  renderer.shadowMap.type = THREE.PCFSoftShadowMap;

  // CAMERA
  const fov = 35;
  const aspect = window.innerWidth / window.innerHeight;
  const near = 0.1;
  const far = 3000;
  camera = new THREE.PerspectiveCamera(fov, aspect, near, far);
  camera.position.set(0, 120, 40);

  controls = new OrbitControls(camera, canvas);
  controls.target.set(0, 105, 0);
  controls.enabled = false; // the user cannot move the prospective
  controls.update();

  // SCENE
  const scene = new THREE.Scene();
  if(night){
    const loader = new THREE.TextureLoader();
    scene.background = loader.load('assets/textures/night_sky.jpg');
    // color switch of the score to make it visible at night
    document.getElementById("score").style.color = "white";
  } else scene.background = new THREE.Color('skyblue');

  // HIERARCHICAL MODEL of the World
  var hModel = new THREE.Group();
  WORLD.createWorld(scene, hModel, night, levelToPlay);
  scene.add(hModel);

  // MAIN CHARACTER bunny-bear
  if(levelToPlay == 1){
    CHARACTER_bunny.initialize_bunny(scene, hModel, night);
  } else if(levelToPlay == 2){
    CHARACTER_bear.initialize_bear(scene, hModel, night);
  } else if(levelToPlay == 3){
    CHARACTER_man.initialize_man(scene, hModel);
  }

  // SOUNDS
  var sound_listener = new THREE.AudioListener();
  camera.add(sound_listener);

  sound_audio = new THREE.Audio(sound_listener);
  sound_audio2 = new THREE.Audio(sound_listener);
  sound_audio3 = new THREE.Audio(sound_listener);
  sound_audio4 = new THREE.Audio(sound_listener);

  var soundLoader = new THREE.AudioLoader(WORLD.loadingManager);
  soundLoader.load( 'audio/1-06EntertheGalaxy.mp3', function( buffer ) {
    sound_audio.setBuffer(buffer);
    sound_audio.setLoop(true);
    sound_audio.setVolume(1.0);
    sound_audio.play();
  });

  var soundLoader2 = new THREE.AudioLoader(WORLD.loadingManager);
  soundLoader2.load( 'audio/lose03.mp3', function( buffer ) {
    sound_audio2.setBuffer(buffer);
    sound_audio2.setLoop(false);
    sound_audio2.setVolume(1.0);
  });

  var soundLoader3 = new THREE.AudioLoader(WORLD.loadingManager);
  if(levelToPlay==1){
    soundLoader3.load( 'audio/mixkit-player-jumping-in-a-video-game-2043.wav', function( buffer ) {
      sound_audio3.setBuffer(buffer);
      sound_audio3.setLoop(false);
      sound_audio3.setVolume(1.0);
    });
  }else if(levelToPlay==2){
    soundLoader3.load( 'audio/rushinginthesnow-trim&fast.mp3', function( buffer ) {
      sound_audio3.setBuffer(buffer);
      sound_audio3.setLoop(false);
      sound_audio3.setVolume(1.0);
    });
  }

  var soundLoader4 = new THREE.AudioLoader(WORLD.loadingManager);
  soundLoader4.load( 'audio/mixkit-fairy-arcade-sparkle-866.wav', function( buffer ) {
    sound_audio4.setBuffer(buffer);
    sound_audio4.setLoop(false);
    sound_audio4.setVolume(1.0);
  });

  function render() {
    if (resizeRendererToDisplaySize(renderer)) {
      const canvas = renderer.domElement;
      camera.aspect = canvas.clientWidth / canvas.clientHeight;
      camera.updateProjectionMatrix();
    }

    if (CARS.dead_hero) setInterval(function(){y_target += 0.4; move_camera_up(controls, y_target);}, 15);
    if (levelToPlay == 2 && CHARACTER_bear.dead_bear) setInterval(function(){y_target += 0.4; move_camera_up(controls, y_target);}, 15);
    else if (levelToPlay == 3 && CHARACTER_man.dead_baby) setInterval(function(){y_target += 0.4; move_camera_up(controls, y_target);}, 25);

    CARS.animate_cars();
    CARS.check_collisions(levelToPlay);

    animate_coins(WORLD.coinsCollisions);

    if (levelToPlay == 2) {
      if (CHARACTER_bear.go_down && !CHARACTER_bear.go_left && !CHARACTER_bear.go_right) CHARACTER_bear.check_collisions_onice(0);
      else if (CHARACTER_bear.go_down && CHARACTER_bear.go_left) CHARACTER_bear.check_collisions_onice(-1);
      else if (CHARACTER_bear.go_down && CHARACTER_bear.go_right) CHARACTER_bear.check_collisions_onice(1);
    } else if (levelToPlay == 3) {
      if(CHARACTER_man.start && !CHARACTER_man.go_left && !CHARACTER_man.go_right) CHARACTER_man.check_collisions_around(0);
      else if (CHARACTER_man.start && CHARACTER_man.go_left) CHARACTER_man.check_collisions_around(-1);
      else if (CHARACTER_man.start && CHARACTER_man.go_right) CHARACTER_man.check_collisions_around(1);
    }

    if (levelToPlay == 1 && CHARACTER_bunny.coin_collision) OBJECTS.generate_new_coin(scene, hModel);
    else if (levelToPlay == 2 && CHARACTER_bear.coin_collision) OBJECTS.generate_new_coin(scene, hModel);
    else if (levelToPlay == 3 && CHARACTER_man.coin_collision) OBJECTS.generate_new_coin(scene, hModel);

    renderer.render(scene, camera);

    id = requestAnimationFrame(render);
  }

  id = requestAnimationFrame(render);
}

main();
