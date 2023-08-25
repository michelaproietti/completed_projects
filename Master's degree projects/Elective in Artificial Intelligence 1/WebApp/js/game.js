import * as THREE from 'https://threejsfundamentals.org/threejs/resources/threejs/r127/build/three.module.js';
import {OrbitControls} from 'https://threejsfundamentals.org/threejs/resources/threejs/r127/examples/jsm/controls/OrbitControls.js';

import * as CLIENT from './client.js'

import * as WORLD from './models.js'
import * as UTILS from './utils.js'

let scene;
export let levelToPlay=1;
export let id;
export let controls, camera;
export let permessoPerMuoversi;

export const canvas = document.querySelector('#c');
export const renderer = new THREE.WebGLRenderer({canvas});


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

export function changePermesso(){
  permessoPerMuoversi = true;
}

var sendData_RuleViolation = "RuleViolation_"
var sendData_EmptyRod = "EmptyRod_";

function sendDataDiskPositions(){
  var sendData_DiskPositions = "OK_"+leftStaff.toString()+"_"+centerStaff.toString()+"_"+rightStaff.toString();
  CLIENT.wsrobot_send(sendData_DiskPositions);
}

sessionStorage.setItem('button_Choice_State', '0');
sessionStorage.setItem('disk_from', '');
sessionStorage.setItem('disk_to', '');

var leftStaff = [];
var centerStaff = [];
var rightStaff = [];
let distance_step = canvas.width * 9 / 100;  //distance between two adjacent rods (left-center or center-right).
                                         // Multiply by 2 if you want the distance between the rods left-center.
var go_up=true;
var go_lateral = false;
var inc_x = 0;
let mov;
let Animation_in_progress = false; 



// idx_disk is the disk we want to animate/move
// direction is -1 (left) or +1 (right)
// num_steps is 1 or 2 if we have to move to an adjacent rod or we have to skip one rod
// rod_to is the rod on witch I want to place the disk
function moveDisk(idx_disk, direction, num_steps, rod_to){

  let max_height = 3 + WORLD.thickness*17 - WORLD.cylinderHeight/2; //maximum height that a disk will reach 
                                                                    //when it has to move to another rod

  
  let disk_subject = scene.getObjectByName("disk"+idx_disk.toString());
  //console.log(disk_subject);

  var lateral_step;
  if(num_steps == 1){
    lateral_step = distance_step;
  } else {
    lateral_step = 2*distance_step;
  }

  var num_disks_on_rod = rod_to.length;
  //y value at which the disk has to stop when going down: 
  var heigth_down_to_reach = 3 + WORLD.thickness*(2*num_disks_on_rod -1) - WORLD.cylinderHeight/2; 

  var old_x_pos_disk = disk_subject.position.x;  //current position along x axis
  go_up = true;
  go_lateral = false;
  inc_x = 0;

  Animation_in_progress = true;
  mov = setInterval(function () {animate(disk_subject, direction, max_height, lateral_step, heigth_down_to_reach, old_x_pos_disk)}, 5);
  
  return;
}


function checkVictory(){
  return ((leftStaff.length==0) && (centerStaff.length==0))
}

function animate(disk_subject, direction, max_height, lateral_step, heigth_down_to_reach, old_x_pos_disk){

  // 1: vertical movement of the disk towards UP

  /*
  console.log("GO LATERAL = ", go_lateral);
  console.log("current x = ", disk_subject.position.x);
  console.log("old x = ", old_x_pos_disk);
  console.log("GO UP = ", go_up);
  console.log("current y = ", disk_subject.position.y);
  console.log("max y = ", max_height);
  console.log("y down = ", heigth_down_to_reach);

  console.log("DIRECTION === ", direction);
  */
  if (go_up && !go_lateral && disk_subject.position.y < max_height){
    //console.log("1");
    disk_subject.position.y += 0.1;

    if (disk_subject.position.y >= max_height){
      go_up = false;
      go_lateral = true;
    }

  }

  // 2: lateral movement
  // I move laterally until I have covered the whole distance 'lateral_step'
  else if (go_lateral && !go_up && inc_x < lateral_step){     //Math.abs(disk_subject.position.x - old_x_pos_disk)
    //console.log("2");
    if(direction == "right"){ // movement towards right
      disk_subject.position.x  += 0.1; 
      inc_x += 0.1;
    } else {  // movement towards left
      disk_subject.position.x  -= 0.1; 
      inc_x += 0.1;
    }

    if (inc_x >= lateral_step){
      go_lateral = false;
    }

  }

  // 3: vertical movement of the disk towards DOWN
  else if(!go_up && !go_lateral && disk_subject.position.y > heigth_down_to_reach){
    //console.log("3");
    disk_subject.position.y -= 0.1;
    if (disk_subject.position.y <= heigth_down_to_reach){
      clearInterval(mov);

      Animation_in_progress = false;

      if(checkVictory()){
        CLIENT.wsrobot_send("Victory");
        disableButtons();
        $(infoBanner).text("VICTORY");
        $(infoBanner).attr("font-size", "26px");
        $(infoBanner).attr("color", "green");
      }else{
        if(isPlannerMoving==false){
          isPlannerMoving=true;
          sendDataDiskPositions();
          $(infoBanner).text("Now It's Pepper Turn!");
        }else{
          isPlannerMoving=false;
          CLIENT.wsrobot_send("ActionDone");
          setTimeout(function (){
            enableButtons();         
          }, 500);
        }
      }
    }
  }

}


let timerPepper

function checkFlag(from_place, from_number, to_place, to_number) {
  if(Animation_in_progress == false){
      clearInterval(timerPepper);
      if(from_place=="loc"){
        if(from_number=="1"){
          sessionStorage.setItem('disk_from', 'left')
        }else if(from_number=="2"){
          sessionStorage.setItem('disk_from', 'center')
        }else if(from_number=="3"){
          sessionStorage.setItem('disk_from', 'right')
        }
      }else{ // Significa che è un disk e devo trovare dove si trova
        if(leftStaff.includes(parseInt(from_number))){
          sessionStorage.setItem('disk_from', 'left')
        }else if(centerStaff.includes(parseInt(from_number))){
          sessionStorage.setItem('disk_from', 'center')
        }else if(rightStaff.includes(parseInt(from_number))){
          sessionStorage.setItem('disk_from', 'right')
        }
      }

      if(to_place=="loc"){
        if(to_number=="1"){
          sessionStorage.setItem('disk_to', 'left')
          CLIENT.wsrobot_send("moveToLeft");
        }else if(to_number=="2"){
          sessionStorage.setItem('disk_to', 'center')
          if(sessionStorage.getItem("disk_from")=="left"){
            CLIENT.wsrobot_send("moveToRight");
          }else{
            CLIENT.wsrobot_send("moveToLeft");
          }
        }else if(to_number=="3"){
          sessionStorage.setItem('disk_to', 'right')
          CLIENT.wsrobot_send("moveToRight");
        }
      }else{ // Significa che è un disk e devo trovare dove si trova
        if(leftStaff.includes(parseInt(to_number))){
          sessionStorage.setItem('disk_to', 'left')
          CLIENT.wsrobot_send("moveToLeft");
        }else if(centerStaff.includes(parseInt(to_number))){
          sessionStorage.setItem('disk_to', 'center')
          if(sessionStorage.getItem("disk_from")=="left"){
            CLIENT.wsrobot_send("moveToRight");
          }else{
            CLIENT.wsrobot_send("moveToLeft");
          }
        }else if(rightStaff.includes(parseInt(to_number))){
          sessionStorage.setItem('disk_to', 'right')
          CLIENT.wsrobot_send("moveToRight");
        }
      }
      checkCoherenceGame();
  }
}


var isPlannerMoving = false;

export function moveForPlanner(thingToDo){
    thingToDo = thingToDo.split(", ")
    var who = thingToDo[0]
    var who_split = who.split("_")
    var from = thingToDo[1]
    var from_split = from.split("_")
    var to = thingToDo[2]
    var to_split = to.split("_")
    isPlannerMoving=true

    timerPepper = setInterval(function () {checkFlag(from_split[0], from_split[1], to_split[0], to_split[1])}, 500);
}




function checkCoherenceGame(){
    /*
    console.log(leftStaff);
    console.log(centerStaff);
    console.log(rightStaff);
    console.log(sessionStorage.getItem('disk_from'));
    console.log(sessionStorage.getItem('disk_to'));
    console.log("Processing:")
    */

    if(Animation_in_progress){
      console.log("Attendi! Già una mossa in corso!");
    }

    // Controllo se la partenza è left
    if(sessionStorage.getItem('disk_from') == "left" && !Animation_in_progress){
      //In primis controllo che la partenza non sia vuota
      if(leftStaff.length==0){
        console.log("Errore! Asta Vuota!");
        enableButtons();
        CLIENT.wsrobot_send(sendData_EmptyRod);
      }else{
        // Ora controllo se l'arrivo è center
        if(sessionStorage.getItem('disk_to') == "center"){
          // Ora controllo che l'arrivo sia vuoto (e nel caso andrebbe bene mettere qualsiasi cosa) 
          // oppure che il suo ultimo elemento sia più grande di quello da mettere
          if(centerStaff.length==0 || centerStaff[centerStaff.length-1]>leftStaff[leftStaff.length-1]){
            centerStaff.push(leftStaff[leftStaff.length-1]);  // insert the disk on the center rod
            leftStaff.pop();   // remove the disk from the left rod
            moveDisk(centerStaff[centerStaff.length-1], "right", 1, centerStaff);  //from left to center
          }else{
            console.log("Errore! Passaggio non permesso perché viola le regole del gioco");
            enableButtons();
            CLIENT.wsrobot_send(sendData_RuleViolation);
          }
        // Ora controllo se l'arrivo è right
        }else{
          // Ora controllo che l'arrivo sia vuoto (e nel caso andrebbe bene mettere qualsiasi cosa) 
          // oppure che il suo ultimo elemento sia più grande di quello da mettere
          if(rightStaff.length==0 || rightStaff[rightStaff.length-1]>leftStaff[leftStaff.length-1]){
            rightStaff.push(leftStaff[leftStaff.length-1]);
            leftStaff.pop();
            moveDisk(rightStaff[rightStaff.length-1], "right", 2, rightStaff);  //from left to right
          }else{
            console.log("Errore! Passaggio non permesso perché viola le regole del gioco");
            enableButtons();
            CLIENT.wsrobot_send(sendData_RuleViolation);
          }
        }
      }
    }

    // Controllo se la partenza è center
    if(sessionStorage.getItem('disk_from') == "center"  && !Animation_in_progress){
      //In primis controllo che la partenza non sia vuota
      if(centerStaff.length==0){
        console.log("Errore! Asta Vuota!");
        enableButtons();
        CLIENT.wsrobot_send(sendData_EmptyRod);
      }else{
        // Ora controllo se l'arrivo è left
        if(sessionStorage.getItem('disk_to') == "left"){
          // Ora controllo che l'arrivo sia vuoto (e nel caso andrebbe bene mettere qualsiasi cosa) 
          // oppure che il suo ultimo elemento sia più grande di quello da mettere
          if(leftStaff.length==0 || leftStaff[leftStaff.length-1]>centerStaff[centerStaff.length-1]){
            leftStaff.push(centerStaff[centerStaff.length-1]);
            centerStaff.pop();
            moveDisk(leftStaff[leftStaff.length-1], "left", 1, leftStaff);  //from center to left
          }else{
            console.log("Errore! Passaggio non permesso perché viola le regole del gioco");
            enableButtons();
            CLIENT.wsrobot_send(sendData_RuleViolation);
          }
        // Ora controllo se l'arrivo è right
        }else{
          // Ora controllo che l'arrivo sia vuoto (e nel caso andrebbe bene mettere qualsiasi cosa) 
          // oppure che il suo ultimo elemento sia più grande di quello da mettere
          if(rightStaff.length==0 || rightStaff[rightStaff.length-1]>centerStaff[centerStaff.length-1]){
            rightStaff.push(centerStaff[centerStaff.length-1]);
            centerStaff.pop();
            moveDisk(rightStaff[rightStaff.length-1], "right", 1, rightStaff);  //from center to right
          }else{
            console.log("Errore! Passaggio non permesso perché viola le regole del gioco");
            enableButtons();
            CLIENT.wsrobot_send(sendData_RuleViolation);
          }
        }
      }
    }

    // Controllo se la partenza è right
    if(sessionStorage.getItem('disk_from') == "right"  && !Animation_in_progress){
      //In primis controllo che la partenza non sia vuota
      if(rightStaff.length==0){
        console.log("Errore! Asta Vuota!");
        enableButtons();
        CLIENT.wsrobot_send(sendData_EmptyRod);
      }else{
        // Ora controllo se l'arrivo è left
        if(sessionStorage.getItem('disk_to') == "left"){
          // Ora controllo che l'arrivo sia vuoto (e nel caso andrebbe bene mettere qualsiasi cosa) 
          // oppure che il suo ultimo elemento sia più grande di quello da mettere
          if(leftStaff.length==0 || leftStaff[leftStaff.length-1]>rightStaff[rightStaff.length-1]){
            leftStaff.push(rightStaff[rightStaff.length-1]);
            rightStaff.pop();
            moveDisk(leftStaff[leftStaff.length-1], "left", 2, leftStaff);  //from right to left
          }else{
            console.log("Errore! Passaggio non permesso perché viola le regole del gioco");
            enableButtons();
            CLIENT.wsrobot_send(sendData_RuleViolation);
          }
        // Ora controllo se l'arrivo è center
        }else{
          // Ora controllo che l'arrivo sia vuoto (e nel caso andrebbe bene mettere qualsiasi cosa) 
          // oppure che il suo ultimo elemento sia più grande di quello da mettere
          if(centerStaff.length==0 || centerStaff[centerStaff.length-1]>rightStaff[rightStaff.length-1]){
            centerStaff.push(rightStaff[rightStaff.length-1]);
            rightStaff.pop();
            moveDisk(centerStaff[centerStaff.length-1], "left", 1, centerStaff);  //from right to center
          }else{
            console.log("Errore! Passaggio non permesso perché viola le regole del gioco");
            enableButtons();
            CLIENT.wsrobot_send(sendData_RuleViolation);
          }
        }
      }
    }

    sessionStorage.setItem('disk_from', '');
    sessionStorage.setItem('disk_to', '');

    /*
    console.log(leftStaff);
    console.log(centerStaff);
    console.log(rightStaff);
    console.log(sessionStorage.getItem('disk_from'));
    console.log(sessionStorage.getItem('disk_to'));
    console.log("#####")
    */

}

function resetStyleButton(){
  $("#leftChoice").css('background', '#b198fc');
  $("#leftChoice").css('border-color', '#7f6db4');
  $("#centerChoice").css('background', '#b198fc');
  $("#centerChoice").css('border-color', '#7f6db4');
  $("#rightChoice").css('background', '#b198fc');
  $("#rightChoice").css('border-color', '#7f6db4');
  $("#leftChoice").html('Pick');
  $("#centerChoice").html('Pick');
  $("#rightChoice").html('Pick');
}

function changeStyleButton(){
  $("#leftChoice").css('background', '#7283f2');
  $("#leftChoice").css('border-color', '#304af2');
  $("#centerChoice").css('background', '#7283f2');
  $("#centerChoice").css('border-color', '#304af2');
  $("#rightChoice").css('background', '#7283f2');
  $("#rightChoice").css('border-color', '#304af2');
}

function disableButtons(){
  $(leftChoice).prop("disabled",true);
  $(centerChoice).prop("disabled",true);
  $(rightChoice).prop("disabled",true);
  $(infoBanner).text("Wait!");
}

function enableButtons(){
  $(leftChoice).prop("disabled",false);
  $(centerChoice).prop("disabled",false);
  $(rightChoice).prop("disabled",false);
  $(infoBanner).text("It's Your Turn!");
}

$(document).ready(function() {
  //La prima cosa che faccio è connettermi al server
  CLIENT.wsrobot_init(9020)

  $(infoBanner).text("It's Your Turn!");

  $("#back").click(function(){
      CLIENT.wsrobot_quit()
      location.href='./main.html'
  });
  $("#exit").click(function(){
    CLIENT.wsrobot_quit()
    location.href='./starting_page.html'
});
  $("#restart").click(function(){
      CLIENT.wsrobot_quit()
      location.reload()
  });
  $("#leftChoice").click(function(){
    if(sessionStorage.getItem('button_Choice_State')==0){
      sessionStorage.setItem('button_Choice_State', '1');
      $("#leftChoice").html('Undo');
      $("#centerChoice").html('Drop');
      $("#rightChoice").html('Drop');
      changeStyleButton();
      $("#leftChoice").css('background', '#f2b272');
      $("#leftChoice").css('border-color', '#f08011');
      sessionStorage.setItem('disk_from', 'left');
    }else{
      if(sessionStorage.getItem('disk_from')=="left"){
        sessionStorage.setItem('disk_from', '');
      }else{
        sessionStorage.setItem('disk_to', 'left');
        disableButtons();
        checkCoherenceGame();
      }
      sessionStorage.setItem('button_Choice_State', '0');
      resetStyleButton();
    }
  });
  
  $("#centerChoice").click(function(){
    if(sessionStorage.getItem('button_Choice_State')==0){
      sessionStorage.setItem('button_Choice_State', '1');
      $("#leftChoice").html('Drop');
      $("#centerChoice").html('Undo');
      $("#rightChoice").html('Drop');
      changeStyleButton();
      $("#centerChoice").css('background', '#f2b272');
      $("#centerChoice").css('border-color', '#f08011');
      sessionStorage.setItem('disk_from', 'center');
    }else{
      if(sessionStorage.getItem('disk_from')=="center"){
        sessionStorage.setItem('disk_from', '');
      }else{
        sessionStorage.setItem('disk_to', 'center');
        disableButtons();
        checkCoherenceGame();
      }
      sessionStorage.setItem('button_Choice_State', '0');
      resetStyleButton();
    }
  });
  $("#rightChoice").click(function(){
    if(sessionStorage.getItem('button_Choice_State')==0){
      sessionStorage.setItem('button_Choice_State', '1');
      $("#leftChoice").html('Drop');
      $("#centerChoice").html('Drop');
      $("#rightChoice").html('Undo');
      changeStyleButton();
      $("#rightChoice").css('background', '#f2b272');
      $("#rightChoice").css('border-color', '#f08011');
      sessionStorage.setItem('disk_from', 'right');
    }else{
      if(sessionStorage.getItem('disk_from')=="right"){
        sessionStorage.setItem('disk_from', '');
      }else{
        sessionStorage.setItem('disk_to', 'right');
        disableButtons();
        checkCoherenceGame();
      }
      sessionStorage.setItem('button_Choice_State', '0');
      resetStyleButton();
    }
  });

  sessionStorage.setItem('tutorial_visibility', 'false');
  sessionStorage.setItem('button_Choice_State', '0');
});


function main() {
  permessoPerMuoversi = false;
  // Reading Local Storage

  // Setting the level to play
  let difficulty = sessionStorage.getItem('difficulty');
  if (difficulty != 0) {
    if (difficulty > 3) {
      levelToPlay=1;
      leftStaff=[3, 2, 1];
    } else if (difficulty == 3) {
      levelToPlay=2;
      leftStaff=[5, 4, 3, 2, 1];
    } else {
      levelToPlay=3;
      leftStaff=[7, 6, 5, 4, 3, 2, 1];
    }
  }
  else if(sessionStorage.getItem("levelPass")=="easy"){
    levelToPlay=1;
    leftStaff=[3, 2, 1];
  } 
  else if(sessionStorage.getItem("levelPass")=="medium"){
    levelToPlay=2;
    leftStaff=[5, 4, 3, 2, 1];
  } 
  else if(sessionStorage.getItem("levelPass")=="hard"){
    levelToPlay=3;
    leftStaff=[7, 6, 5, 4, 3, 2, 1];
  } 

  const pi = Math.PI;
  renderer.shadowMap.enabled = true;
  renderer.shadowMap.type = THREE.PCFSoftShadowMap;

  // CAMERA
  const fov = 35;
  const aspect = window.innerWidth / window.innerHeight;
  const near = 0.1;
  const far = 3000;
  camera = new THREE.PerspectiveCamera(fov, aspect, near, far);
  camera.position.set(0, 10, 120);

  controls = new OrbitControls(camera, canvas);
  controls.target.set(0, 0, 0);
  controls.enabled = false; // the user cannot move the prospective
  controls.update();

  // SCENE
  scene = new THREE.Scene();
  scene.name = "Scene";
  scene.background = new THREE.Color(0xCAFFFF);

  // HIERARCHICAL MODEL of the World
  var hModel = new THREE.Group();
  hModel.name = "Game"
  WORLD.createWorld(scene, hModel, levelToPlay);
  scene.add(hModel);

  //console.log(dumpObject(scene).join('\n'));

  function render() {
    if (UTILS.resizeRendererToDisplaySize(renderer)) {
      const canvas = renderer.domElement;
      camera.aspect = canvas.clientWidth / canvas.clientHeight;
      camera.updateProjectionMatrix();
    }

    renderer.render(scene, camera);

    if(CLIENT.connessioneStabilita==0){
      window.alert("Connection problems! Returning automatically to the main menu")
      location.href='./main.html'
      CLIENT.connessioneStabilita=-2
    }

    id = requestAnimationFrame(render);
  }

  id = requestAnimationFrame(render);
}

main();
