import * as THREE from 'https://threejs.org/build/three.module.js';
import {OBJLoader} from 'https://threejs.org/examples/jsm/loaders/OBJLoader.js';
import {MTLLoader} from 'https://threejs.org/examples/jsm/loaders/MTLLoader.js';

import * as OBJECTS from './objects.js'
import * as GAME from './game.js'

export let cars_ltr = [];
export let cars_rtl = [];
export let cars = [];

export let bigObstaclesCollision = [];
export let smallObstaclesCollision = [];
export let iceCollisions = [];
export let coinsCollisions = [];

export let ice_start = [];
export let ice_end = [];

let progress = document.createElement('progress_bar');
let progressBar = document.createElement('progress_bar');
progress.appendChild(progressBar);
document.body.appendChild(progress);

export let loadingManager = new THREE.LoadingManager();
//loadingManager.onProgress = function (item, loaded, total) {
  //progressBar.style.width = (loaded / total * 100) + '%';
  //console.log((loaded / total * 100) + '%');
//};
loadingManager.onLoad = function ( ) {
  $("#progress").hide();
  GAME.changePermesso();
  $("#c").fadeIn(1000);
  $("#buttonsArea").fadeIn(1000);
};

// LIGHTS
export function addLights(scene, night) {
  // hemisphere light
  const skyColor = 0xB1E1FF;  // light blue
  const groundColor = 0xB97A20;  // brownish orange
  const color = 0xFFFFFF; // white

  let intensity_hem = 1;
  if (night) intensity_hem = 0.5;

  const hemisphere_light = new THREE.AmbientLight(color, intensity_hem);
  scene.add(hemisphere_light);


  let intensity_dir = 1;
  if (night) intensity_dir = 0.15;
  // directional light
  const directional_light = new THREE.DirectionalLight(color, intensity_dir);
  directional_light.position.set(100, 100, 100);
  directional_light.castShadow = true;
  directional_light.shadow.mapSize.width = 100 * window.devicePixelRatio;
  directional_light.shadow.mapSize.height = 100 * window.devicePixelRatio;
  directional_light.shadow.camera = new THREE.OrthographicCamera( -100, 100, 100, -100, 0.1, 3000); 
  directional_light.shadow.camera.near = 0.1;
  directional_light.shadow.camera.far = 3000;
  directional_light.shadow.camera.fov = 1000;

  scene.add(directional_light);
}


export function createCylinder(model, level) {
  const radiusTop = 100;
  const radiusBottom = 100;
  const cylinderHeight = 200;
  const baseSides = 1024;
  const textureSize = 1000;

  let material;
  if(level==1){ //grass
    const loader = new THREE.TextureLoader(loadingManager);
    const texture = loader.load('./assets/textures/grass-texture.jpg');
    texture.wrapS = THREE.RepeatWrapping;
    texture.wrapT = THREE.RepeatWrapping;
    texture.magFilter = THREE.NearestFilter;
    const repeats = textureSize / 2;
    texture.repeat.set(repeats, repeats);
  
    material = new THREE.MeshPhongMaterial({
      map: texture,
      side: THREE.DoubleSide,
    });

  } else if(level==2){  //snow
    const textureLoader = new THREE.TextureLoader(loadingManager);
    material = new THREE.MeshStandardMaterial({color:0x8B8B8B, flatShading:THREE.FlatShading} )
    material.bumpMap = textureLoader.load('./assets/textures/bump1.png');   
    material.bumpScale = 1;

  } else if(level==3){ //green slime
    material = new THREE.MeshPhongMaterial({color: 0x31FF00});
  }

  const geometry = new THREE.CylinderGeometry(radiusTop, radiusBottom, cylinderHeight, baseSides);

  const cylWorld = new THREE.Mesh(geometry, material);
  cylWorld.position.set(0, 0, 0);
  cylWorld.rotation.z = Math.PI/2;
  cylWorld.receiveShadow = true;
  cylWorld.name = "cylWorld";
  model.add(cylWorld);
}


export function add3DObject(
  scene,            // scene
  model,            // hierarchical model
  name,             // name of the object
  mtlFile,          // file .mtl
  objFile,          // file .obj
  position,         // position in 3D space (3 elements array)
  rotation,         // orientation (3 elements array)
  scale,            // scale (3 elements array)
  doubleSideBool,   // true/false to put double side material
  hModelBool,       // true if the object in the hierachical model, false otherwise
  obstacle=0,       // 1 if it is an obstacle
  vehicle=0,        // if we are adding a car model, then it will be equal to its direction along x
  ice=-1            // 0 if we are on the first slice of ice, 1 if we are on the last one
) {
  const mtlLoader = new MTLLoader(loadingManager);
  mtlLoader.load(mtlFile, (mtl) => {
    mtl.preload();
    const objLoader = new OBJLoader(loadingManager);
    if (doubleSideBool) mtl.materials.Material.side = THREE.DoubleSide;
    objLoader.setMaterials(mtl);
    objLoader.load(objFile, (root) => {
      root.name = name;
      root.position.set(position[0], position[1], position[2]);
      root.rotation.set(rotation[0], rotation[1], rotation[2]);
      root.scale.set(scale[0], scale[1], scale[2]);

      root.traverse( function( node ) { if ( node instanceof THREE.Mesh ) { node.castShadow = true; } } );

      if (vehicle != 0) {
        cars.push(root);
        if (vehicle == -1) cars_rtl.push(root);
        else if (vehicle == 1) cars_ltr.push(root);
      }
      if (obstacle == 1) smallObstaclesCollision.push(root);
      else if (obstacle == 2) bigObstaclesCollision.push(root);
      else if (obstacle == 3) iceCollisions.push(root);
      else if (obstacle == 4) coinsCollisions.push(root);
      if (hModelBool) model.add(root);
      if (!hModelBool) scene.add(root);
      if (ice == 0) ice_start.push(root);
      else if (ice == 1) ice_end.push(root);
    });
  });
}

function createHierarchicalModel(model, level){
  //Level 1
  var smallObstaclesModel = new THREE.Group();
  smallObstaclesModel.name = "smallObstaclesModel";
  var bigObstaclesModel = new THREE.Group();
  bigObstaclesModel.name = "bigObstaclesModel";
  var roadsModel = new THREE.Group();
  roadsModel.name = "roadsModel";
  var vehiclesModel = new THREE.Group();
  vehiclesModel.name = "vehiclesModel";
  var edgesModel = new THREE.Group();
  edgesModel.name = "edgesModel";
  var coinsModel = new THREE.Group();
  coinsModel.name = "coinsModel";
  model.add(smallObstaclesModel);
  model.add(bigObstaclesModel);
  model.add(roadsModel);
  model.add(vehiclesModel);
  model.add(edgesModel);
  model.add(coinsModel);

  //Trees
  if(level==1){  //easy
    var smallObs_Lowpoly_tree_sample = new THREE.Group();
    smallObs_Lowpoly_tree_sample.name = "smallObs_Lowpoly_tree_sample";
    smallObstaclesModel.add(smallObs_Lowpoly_tree_sample);
    var smallObs_BirchTree_1 = new THREE.Group();
    smallObs_BirchTree_1.name = "smallObs_BirchTree_1";
    smallObstaclesModel.add(smallObs_BirchTree_1);
    var smallObs_BushBerries_1 = new THREE.Group();
    smallObs_BushBerries_1.name = "smallObs_BushBerries_1";
    smallObstaclesModel.add(smallObs_BushBerries_1);

  } else if(level==2) {  //medium
    var smallObs_BirchTree_Dead_Snow_1 = new THREE.Group();
    smallObs_BirchTree_Dead_Snow_1.name = "smallObs_BirchTree_Dead_Snow_1";
    smallObstaclesModel.add(smallObs_BirchTree_Dead_Snow_1);
    var smallObs_BirchTree_Snow_1 = new THREE.Group();
    smallObs_BirchTree_Snow_1.name = "smallObs_BirchTree_Snow_1";
    smallObstaclesModel.add(smallObs_BirchTree_Snow_1);
    var smallObs_CommonTree_Snow_4 = new THREE.Group();
    smallObs_CommonTree_Snow_4.name = "smallObs_CommonTree_Snow_4";
    smallObstaclesModel.add(smallObs_CommonTree_Snow_4 );
    var smallObs_PineTree_Snow_3 = new THREE.Group();
    smallObs_PineTree_Snow_3.name = "smallObs_PineTree_Snow_3";
    smallObstaclesModel.add(smallObs_PineTree_Snow_3);
    var smallObs_candyCane = new THREE.Group();
    smallObs_candyCane.name = "smallObs_candyCane";
    smallObstaclesModel.add(smallObs_candyCane);
    var smallObs_present = new THREE.Group();
    smallObs_present.name = "smallObs_present";
    smallObstaclesModel.add(smallObs_present);
    var smallObs_candyCaneMint = new THREE.Group();
    smallObs_candyCaneMint.name = "smallObs_candyCaneMint";
    smallObstaclesModel.add(smallObs_candyCaneMint);

  } else if (level == 3) {  // hard
    var smallObs_candyBag = new THREE.Group();
    smallObs_candyBag.name = 'smallObs_candyBag';
    smallObstaclesModel.add(smallObs_candyBag);
    var smallObs_candyBucket = new THREE.Group();
    smallObs_candyBucket.name = 'smallObs_candyBucket';
    smallObstaclesModel.add(smallObs_candyBucket);
    var smallObs_cauldron = new THREE.Group();
    smallObs_cauldron.name = 'smallObs_cauldron';
    smallObstaclesModel.add(smallObs_cauldron);
    var smallObs_cross = new THREE.Group();
    smallObs_cross.name = 'smallObs_cross';
    smallObstaclesModel.add(smallObs_cross);
    var smallObs_lampPost = new THREE.Group();
    smallObs_lampPost.name = 'smallObs_lampPost';
    smallObstaclesModel.add(smallObs_lampPost);
    var smallObs_lollipopA = new THREE.Group();
    smallObs_lollipopA.name = 'smallObs_lollipopA';
    smallObstaclesModel.add(smallObs_lollipopA);
    var smallObs_lollipopB = new THREE.Group();
    smallObs_lollipopB.name = 'smallObs_lollipopB';
    smallObstaclesModel.add(smallObs_lollipopB);
    var smallObs_treeA = new THREE.Group();
    smallObs_treeA.name = 'smallObs_treeA';
    smallObstaclesModel.add(smallObs_treeA);
    var smallObs_treeB = new THREE.Group();
    smallObs_treeB.name = 'smallObs_treeB';
    smallObstaclesModel.add(smallObs_treeB);
    var smallObs_treeC = new THREE.Group();
    smallObs_treeC.name = 'smallObs_treeC';
    smallObstaclesModel.add(smallObs_treeC);
    var smallObs_treeD = new THREE.Group();
    smallObs_treeD.name = 'smallObs_treeD';
    smallObstaclesModel.add(smallObs_treeD);
    var smallObs_trunk = new THREE.Group();
    smallObs_trunk.name = 'smallObs_trunk';
    smallObstaclesModel.add(smallObs_trunk);
    var smallObs_trunkLong = new THREE.Group();
    smallObs_trunkLong.name = 'smallObs_trunkLong';
    smallObstaclesModel.add(smallObs_trunkLong);
  }



  //Buildings
  if(level==1){  //easy
    var bigObs_Building1_Large = new THREE.Group();
    bigObs_Building1_Large.name = "bigObs_Building1_Large";
    bigObstaclesModel.add(bigObs_Building1_Large);
    var bigObs_Building2_Large = new THREE.Group();
    bigObs_Building2_Large.name = "bigObs_Building2_Large";
    bigObstaclesModel.add(bigObs_Building2_Large);
    var bigObs_Building3_Big = new THREE.Group();
    bigObs_Building3_Big.name = "bigObs_Building3_Big";
    bigObstaclesModel.add(bigObs_Building3_Big);
    var bigObs_Building4 = new THREE.Group();
    bigObs_Building4.name = "bigObs_Building4";
    bigObstaclesModel.add(bigObs_Building4);
    var bigObs_House1 = new THREE.Group();
    bigObs_House1.name = "bigObs_House1";
    bigObstaclesModel.add(bigObs_House1);
    var bigObs_Building1_Small = new THREE.Group();
    bigObs_Building1_Small.name = "bigObs_Building1_Small";
    bigObstaclesModel.add(bigObs_Building1_Small);
    var bigObs_Building2_Small = new THREE.Group();
    bigObs_Building2_Small.name = "bigObs_Building2_Small";
    bigObstaclesModel.add(bigObs_Building2_Small);
    var bigObs_Building3_Small = new THREE.Group();
    bigObs_Building3_Small.name = "bigObs_Building3_Small";
    bigObstaclesModel.add(bigObs_Building3_Small);

  } else if(level==2) { //medium
    var bigObs_WoodLog_Snow = new THREE.Group();
    bigObs_WoodLog_Snow.name = "bigObs_WoodLog_Snow";
    bigObstaclesModel.add(bigObs_WoodLog_Snow);
    var bigObs_WoodLog_Moss = new THREE.Group();
    bigObs_WoodLog_Moss.name = "bigObs_WoodLog_Moss";
    bigObstaclesModel.add(bigObs_WoodLog_Moss);
    var bigObs_TreeStump_Snow = new THREE.Group();
    bigObs_TreeStump_Snow.name = "bigObs_TreeStump_Snow";
    bigObstaclesModel.add(bigObs_TreeStump_Snow);
    var bigObs_Rock_Snow_1 = new THREE.Group();
    bigObs_Rock_Snow_1.name = "bigObs_Rock_Snow_1";
    bigObstaclesModel.add(bigObs_Rock_Snow_1);
    var bigObs_Rock_Snow_6 = new THREE.Group();
    bigObs_Rock_Snow_6.name = "bigObs_Rock_Snow_6";
    bigObstaclesModel.add(bigObs_Rock_Snow_6);
    var bigObs_bench = new THREE.Group();
    bigObs_bench.name = "bigObs_bench";
    bigObstaclesModel.add(bigObs_bench);
    var bigObs_snowFort = new THREE.Group();
    bigObs_snowFort.name = "bigObs_snowFort";
    bigObstaclesModel.add(bigObs_snowFort);
    var bigObs_treeDecorated = new THREE.Group();
    bigObs_treeDecorated.name = "bigObs_treeDecorated";
    bigObstaclesModel.add(bigObs_treeDecorated);

  } else if (level == 3) {
    var bigObs_bench = new THREE.Group();
    bigObs_bench.name = "bigObs_bench";
    bigObstaclesModel.add(bigObs_bench);
    var bigObs_benchBroken = new THREE.Group();
    bigObs_benchBroken.name = "bigObs_benchBroken";
    bigObstaclesModel.add(bigObs_benchBroken);
    var bigObs_gravestone = new THREE.Group();
    bigObs_gravestone.name = "bigObs_gravestone";
    bigObstaclesModel.add(bigObs_gravestone);
    var bigObs_gravestoneDecorative = new THREE.Group();
    bigObs_gravestoneDecorative.name = "bigObs_gravestoneDecorative";
    bigObstaclesModel.add(bigObs_gravestoneDecorative);
    var bigObs_pumpkin = new THREE.Group();
    bigObs_pumpkin.name = "bigObs_pumpkin";
    bigObstaclesModel.add(bigObs_pumpkin);
    var bigObs_pumpkinCarved = new THREE.Group();
    bigObs_pumpkinCarved.name = "bigObs_pumpkinCarved";
    bigObstaclesModel.add(bigObs_pumpkinCarved);
    var bigObs_pumpkinCarvedTall = new THREE.Group();
    bigObs_pumpkinCarvedTall.name = "bigObs_pumpkinCarvedTall";
    bigObstaclesModel.add(bigObs_pumpkinCarvedTall);
    var bigObs_pumpkinTall = new THREE.Group();
    bigObs_pumpkinTall.name = "bigObs_pumpkinTall";
    bigObstaclesModel.add(bigObs_pumpkinTall);
  }


  //Vehicles
  if(level==1){
    var vehicle_ambulance = new THREE.Group();
    vehicle_ambulance.name = "vehicle_ambulance";
    vehiclesModel.add(vehicle_ambulance);
    var vehicle_delivery = new THREE.Group();
    vehicle_delivery.name = "vehicle_delivery";
    vehiclesModel.add(vehicle_delivery);
    var vehicle_deliveryFlat = new THREE.Group();
    vehicle_deliveryFlat.name = "vehicle_deliveryFlat";
    vehiclesModel.add(vehicle_deliveryFlat);
    var vehicle_firetruck = new THREE.Group();
    vehicle_firetruck.name = "vehicle_firetruck";
    vehiclesModel.add(vehicle_firetruck);
    var vehicle_garbageTruck = new THREE.Group();
    vehicle_garbageTruck.name = "vehicle_garbageTruck";
    vehiclesModel.add(vehicle_garbageTruck);
    var vehicle_hatchbackSports = new THREE.Group();
    vehicle_hatchbackSports.name = "vehicle_hatchbackSports";
    vehiclesModel.add(vehicle_hatchbackSports);
    var vehicle_police = new THREE.Group();
    vehicle_police.name = "vehicle_police";
    vehiclesModel.add(vehicle_police);
    var vehicle_race = new THREE.Group();
    vehicle_race.name = "vehicle_race";
    vehiclesModel.add(vehicle_race);
    var vehicle_raceFuture = new THREE.Group();
    vehicle_raceFuture.name = "vehicle_raceFuture";
    vehiclesModel.add(vehicle_raceFuture);
    var vehicle_sedan = new THREE.Group();
    vehicle_sedan.name = "vehicle_sedan";
    vehiclesModel.add(vehicle_sedan);
    var vehicle1_sedanSports = new THREE.Group();
    vehicle1_sedanSports.name = "vehicle_sedanSports";
    vehiclesModel.add(vehicle1_sedanSports);
    var vehicle_suv = new THREE.Group();
    vehicle_suv.name = "vehicle_suv";
    vehiclesModel.add(vehicle_suv);
    var vehicle_suvLuxury = new THREE.Group();
    vehicle_suvLuxury.name = "vehicle_suvLuxury";
    vehiclesModel.add(vehicle_suvLuxury);
    var vehicle_taxi = new THREE.Group();
    vehicle_taxi.name = "vehicle_taxi";
    vehiclesModel.add(vehicle_taxi);
    var vehicle_tractor = new THREE.Group();
    vehicle_tractor.name = "vehicle_tractor";
    vehiclesModel.add(vehicle_tractor);
    var vehicle_tractorPolice = new THREE.Group();
    vehicle_tractorPolice.name = "vehicle_tractorPolice";
    vehiclesModel.add(vehicle_tractorPolice);
    var vehicle_tractorShovel = new THREE.Group();
    vehicle_tractorShovel.name = "vehicle_tractorShovel";
    vehiclesModel.add(vehicle_tractorShovel);
    var vehicle_truck = new THREE.Group();
    vehicle_truck.name = "vehicle_truck";
    vehiclesModel.add(vehicle_truck);
    var vehicle_truckFlat = new THREE.Group();
    vehicle_truckFlat.name = "vehicle_truckFlat";
    vehiclesModel.add(vehicle_truckFlat);
    var vehicle_van = new THREE.Group();
    vehicle_van.name = "vehicle_van";
    vehiclesModel.add(vehicle_van);

  } else if(level==2) {
    /*var vehicle_van = new THREE.Group();
    vehicle_van.name = "vehicle_van";
    vehiclesModel.add(vehicle_van); */
    var vehicle_sled = new THREE.Group();
    vehicle_sled.name = "vehicle_sled";
    vehiclesModel.add(vehicle_sled);
    var vehicle_trainLoc = new THREE.Group();
    vehicle_trainLoc.name = "vehicle_trainLoc";
    vehiclesModel.add(vehicle_trainLoc);
    var vehicle_snowman = new THREE.Group();
    vehicle_snowman.name = "vehicle_snowman";
    vehiclesModel.add(vehicle_snowman);
    var vehicle_snowmanFancy = new THREE.Group();
    vehicle_snowmanFancy.name = "vehicle_snowmanFancy";
    vehiclesModel.add(vehicle_snowmanFancy);
    
  } else if (level == 3) {
    var vehicle_Slime = new THREE.Group();
    vehicle_Slime.name = "vehicle_Slime";
    vehiclesModel.add(vehicle_Slime);
  }

  // coins
  var vehicle_Slime = new THREE.Group();
  vehicle_Slime.name = "vehicle_Slime";
  vehiclesModel.add(vehicle_Slime);

}

export function createWorld(scene, model, night, level) {
  addLights(scene, night);

  createCylinder(model, level);

  createHierarchicalModel(model, level);

  OBJECTS.add_all_objects_in_world(scene, model, level);
}