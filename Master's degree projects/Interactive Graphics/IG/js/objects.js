import * as WORLD from './models.js'

export let positionOfSmallObstacles = [];
export let positionOfBigObstacles = [];
export let positionOfRoads = [];

const pi = Math.PI;

// function needed in order to check that the same position isn't occupied by more than one object.
function checkGenerationCollision(x, y, z){
  var checkSmallObstacles;
  checkSmallObstacles = !positionOfSmallObstacles.some(c => c.x<=x+2 && c.x>=x-2 && c.y<=y+2 && c.y>=y-2 && c.z<=z+2 && c.z>=z-2);
  var checkRoads;
  checkRoads = !positionOfRoads.some(c => c.y<=y+2 && c.y>=y-2 && c.z<=z+2 && c.z>=z-2);
  var checkBuilding;
  checkBuilding = !positionOfBigObstacles.some(c => c.x<=x+4 && c.x>=x-4 && c.y<=y+3 && c.y>=y-3 && c.z<=z+4 && c.z>=z-4);
  return checkSmallObstacles && checkRoads && checkBuilding;
}

export function add_all_objects_in_world(scene, model, level){ 
  // number of segments large enough to contain three roads joined side by side: 48
  // maximum number of streets per segment: 3 (to be changed according to the level)
  const locations = getRandomRoadLocations(48, 3);

  for (var i = 0; i < locations.length; i++) {
    var pair = locations[i];
    var deg = pair[0];
    var exist = pair[1];
    let ice_bool=false;  // boolean passed to the 'placeRoad' in order to put the ice or not at level 2

    //the if else is needed to place, where we want in the cylinder, the iced lake
    if(level==2 && ((i>=1 && i<=4) || (i>=40 && i<=44) || (i>=26 && i<=31) || (i>=12 && i<=18))) { //&& i>=15 && i<22) || (level==2 && i>=39 && i<47) ){  //10-22   / 35-47
      ice_bool=true;
      placeRoad(scene, model, deg, level, 1, ice_bool, false, i);
      placeEdge(scene, model, deg, 0, level);
      placeEdge(scene, model, deg+2.0, 0, level);
      placeEdge(scene, model, deg+4.0, 0, level);
      placeEdge(scene, model, deg+6.0, 0, level);

      if(level==2 && (i==4 || i==44 || i==31 || i==18)) {
        continue;
      }

      placeRandomBigObstacle(scene, model, deg, level, ice_bool);
      placeRandomBigObstacle(scene, model, deg+2.5, level, ice_bool);
      placeRandomBigObstacle(scene, model, deg+5.0, level, ice_bool);
    } else {
      if(exist==0){
        // Clove not containing roads (we populate it with various objects)
        placeEdge(scene, model, deg, 0, level);
        placeEdge(scene, model, deg+2.0, 0, level);
        placeEdge(scene, model, deg+4.0, 0, level);
        placeEdge(scene, model, deg+6.0, 0, level);

        if (level != 3) {
          placeRandomSmallObstacle(scene, model, deg, level);
          placeRandomBigObstacle(scene, model, deg, level, ice_bool);
          placeRandomSmallObstacle(scene, model, deg+2.5, level);
          placeRandomSmallObstacle(scene, model, deg+5.0, level);
          placeRandomBigObstacle(scene, model, deg+5.0, level, ice_bool);
        } else {
          placeRandomSmallObstacle(scene, model, deg, level);
          placeRandomBigObstacle(scene, model, deg, level, ice_bool);
          placeRandomSmallObstacle(scene, model, deg+2.5, level);
        }

        placeCoin(scene, model, deg);

      }else if(exist==1){
        // Check for no roads to appear right after the ice
        if(level==2 && (i==0 || i==39 || i==25 || i==11)) {
          placeEdge(scene, model, deg, 0, level);
          placeEdge(scene, model, deg+2.0, 0, level);
          placeEdge(scene, model, deg+4.0, 0, level);
          placeEdge(scene, model, deg+6.0, 0, level);
          placeRandomSmallObstacle(scene, model, deg, level);
          placeRandomBigObstacle(scene, model, deg, level, ice_bool);
          placeRandomSmallObstacle(scene, model, deg+2.5, level);
          continue;
        } 

        // clove containing roads and vehicles
        var nRoads = pair[2];
        let trainOrIceLevel2 = false;
        if (nRoads == 1) {
          placeEdge(scene, model, deg, 1, level);
          placeEdge(scene, model, deg+2.5, 0, level);
          placeEdge(scene, model, deg+4.0, 0, level);
          placeEdge(scene, model, deg+6.0, 0, level);
  
          if(level==2 && Math.random()<=0.85){
            trainOrIceLevel2 = true;
          }
          placeRoad(scene, model, deg, level, 1, ice_bool, trainOrIceLevel2);
        }
        else if (nRoads == 2) {
          placeEdge(scene, model, deg, 1, level);
          placeEdge(scene, model, deg+2.5, 1, level);
          placeEdge(scene, model, deg+4.0, 0, level);
          placeEdge(scene, model, deg+6.0, 0), level;

          placeRoad(scene, model, deg, level, 1, ice_bool);
          placeRoad(scene, model, deg+2.5, level, -1, ice_bool);
        }
        else if (nRoads == 3) {
          placeEdge(scene, model, deg, 1, level);
          placeEdge(scene, model, deg+2.5, 1, level);
          placeEdge(scene, model, deg+5.0, 1, level);

          placeRoad(scene, model, deg, level, 1, ice_bool);
          placeRoad(scene, model, deg+2.5, level, -1, ice_bool);
          if(level==2 && Math.random()<=0.65){
            trainOrIceLevel2 = true;
          }
          placeRoad(scene, model, deg+5.0, level, 1, ice_bool, trainOrIceLevel2);
          //last road met by the hero
        }
      }
    }  
  }
}

function placeEdge(scene, model, deg, id, level) {
  const r = 100;
  const radians = 57.2958;
  const x = 26.6;
  let theta_x = deg / radians;
  const theta_y = 90 / radians;
  let y = r * Math.cos(theta_x);
  let z = r * Math.sin(theta_x);

  let path, edge, scale;

  path = 'assets/models/tunnel/';

  // names and scales
  edge = [
    "wallSingle",
    "tunnel"
  ];

  scale = [1.0, 1.2, 1.0];

  // left wall
  WORLD.add3DObject(
    scene,
    model.getObjectByName("edgesModel"),
    edge[id],
    path + edge[id] + '.mtl',
    path + edge[id] + '.obj',
    [x, y, z],
    [theta_x, -theta_y, 0.0],
    scale,
    false,
    true,
    0,
    0,
    -1
  );

  // right wall
  WORLD.add3DObject(
    scene,
    model.getObjectByName("edgesModel"),
    edge,
    path + edge[id] + '.mtl',
    path + edge[id] + '.obj',
    [-x, y, z],
    [theta_x, theta_y, 0.0],
    scale,
    false,
    true,
    0,
    0,
    -1
  );
}

function placeRandomSmallObstacle(scene, model, deg, level) {
  const r = 100;
  const radians = 57.2958;
  const theta_x = deg / radians;
  let x = randomValue(-24, 24);
  let y = r * Math.cos(theta_x);
  const z = r * Math.sin(theta_x);

  // the following if-else avoids that small obstacles appear too close 
  // to the initial position of the main character (placed at deg=0)
  if ((deg >= 345 || deg < 10) && (deg % 2 == 0) && (x <= 6 || x >= -6)) x = randomValue(-24, -6);
  else if ((deg >= 345 || deg < 10) && (deg % 2 != 0) && (x <= 6 || x >= -6)) x = randomValue(6, 24);

  if(checkGenerationCollision(x,y,z)){
    
    // names and scales
    let smallObstacles;
    let path;
    let space_smallObstacles;

    if(level==1){  // easy
      path = 'assets/models/level1/trees/';
      smallObstacles = [
        ['Lowpoly_tree_sample', [0.23, 0.23, 0.23], -90.0],
        ['BirchTree_1', [2.2, 2.2, 2.2], 0],
        ['BushBerries_1', [1.5, 1.5, 1.5], 0]
      ];
      space_smallObstacles = [ 3, 2, 1 ];

    } else if(level==2){  // medium
      path = 'assets/models/level2/small_obj/';
      smallObstacles = [
        ['BirchTree_Dead_Snow_1', [2.5, 2.5, 2.5], -90.0],
        ['BirchTree_Snow_1', [2.5, 2.5, 2.5], -90.0],
        ['CommonTree_Snow_4', [2.5, 2.5, 2.5], -90.0],
        ['PineTree_Snow_3', [2.5, 2.5, 2.5], -90.0],
        ['candyCane', [15, 15, 15], Math.PI],
        ['candyCaneMint', [15, 15, 15], Math.PI], 
        ['present', [5, 5, 5], 0.0] 
      ];
      space_smallObstacles = [ 3, 3, 2, 4, 2, 2, 3 ];

    } else if (level == 3) { // hard
      path = './assets/models/level3/obstacles/thin/';
      smallObstacles = [
        ['cauldron', [3, 3, 3], 0],
        ['cross', [7, 7, 7], 0],
        ['lampPost', [3, 3, 3], 0],
        ['lollipopA', [75, 75, 75], 0],
        ['lollipopB', [0.1, 0.1, 0.1], 0],
        ['treeA', [3, 3, 3], 0],
        ['treeB', [3, 3, 3], 0],
        ['treeC', [3, 3, 3], 0],
        ['treeD', [3, 3, 3], 0],
        ['trunk', [4, 4, 4], 0],
        ['trunkLong', [4, 4, 4], 0],
      ];

      space_smallObstacles = [4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4];
    }

    const nSmallObstacles = smallObstacles.length;
    const id = getRandomInt(0, nSmallObstacles-1);
    const tree = smallObstacles[id][0];
    const scale = smallObstacles[id][1];
    const theta_y = smallObstacles[id][2];


    WORLD.add3DObject(
      scene,
      model.getObjectByName("smallObs_"+tree),
      tree,
      path + tree + '.mtl',
      path + tree + '.obj',
      [x, y, z],
      [theta_x, theta_y, 0.0],
      scale,
      false,
      true,
      1,
      0,
      -1
    );

    for(let i=-space_smallObstacles[id]; i<=space_smallObstacles[id]; i++){
      for(let j=-2; j<=2; j++){
        for(let k=-1; k<=1; k++){
          positionOfSmallObstacles.push({x:x+i, y:y+j, z:z+k});
        }
      }
    }
  }
}

function placeRandomBigObstacle(scene, model, deg, level, ice_bool) {
  const r = 100;
  const radians = 57.2958;
  const theta = deg / radians;
  let x = randomValue(-20, 20);
  const y = r * Math.cos(theta);
  const z = r * Math.sin(theta);
  let obstacle_type = 2;

  // the following if-else avoids that big obstacles appear too close 
  // to the initial position of the main character (placed at deg=0)
  if ((deg >= 345 || deg < 10) && (deg % 2 == 0) && (x <= 6 || x >= -6)) x = randomValue(6, 20);
  else if ((deg >= 345 || deg < 10) && (deg % 2 != 0) && (x <= 6 || x >= -6)) x = randomValue(-20, -6);

  let path;
  let bigObstacles;
  let space_bigObstacles;

  if(level==1){
    path = 'assets/models/level1/houses/';
    bigObstacles = [
      ['Building1_Large', [1.5, 1.5, 1.5], 0],
      ['Building2_Large', [1.5, 1.5, 1.5], 0],
      ['Building3_Big', [1.5, 1.5, 1.5], 0],
      ['Building4', [1.5, 1.5, 1.5], 0],
      ['House1', [1.5, 1.5, 1.5], 0],
      ['Building1_Small', [1.5, 1.5, 1.5], 0],
      ['Building2_Small', [1.5, 1.5, 1.5], 0],
      ['Building3_Small', [1.5, 1.5, 1.5], 0]
    ];
    space_bigObstacles = [ 12, 10, 7, 7, 3, 3, 5, 5, 5 ];

  } else if(level==2) {
    path = 'assets/models/level2/big_obj/';
    if(!ice_bool){
      bigObstacles = [
        ['WoodLog_Snow', [2.5, 3, 2.5], 80],
        ['WoodLog_Moss', [2.5, 3, 2.5], 80],
        ['TreeStump_Snow', [2.5, 3, 2.5], 80],
        ['Rock_Snow_1', [2.5, 3, 2.5], 80],
        ['Rock_Snow_6', [2.5, 3, 2.5], 80],
        ['bench', [10.0, 5.0, 8.0], Math.PI],
        ['treeDecorated', [5, 5, 5], 0.0],
      ];
      space_bigObstacles = [ 12, 8, 4, 3, 3, 12, 3];

    } else {
      bigObstacles = [
        ['WoodLog_Snow', [2.5, 3, 2.5], 80],
        ['Rock_Snow_1', [5, 5, 5], 80],
        ['Rock_Snow_6', [5, 5, 5], 80] ,
        ['snowFort', [7, 7, 7], Math.PI]
      ];
      space_bigObstacles = [ 12, 4, 4, 12, 6 ];
      obstacle_type = 3;
    }
    
  } else if (level == 3) {
    path = './assets/models/level3/obstacles/';
    bigObstacles = [
      ['gravestone', [1.5, 1.5, 1.5], 0],
      ['pumpkin', [9, 9, 9], 0],
      ['pumpkinTall', [9, 9, 9], 0]
    ];

    space_bigObstacles = [4, 4, 4, 4, 4, 4];
  }


  const nBigObstacles = bigObstacles.length;
  var id = getRandomInt(0, nBigObstacles-1);
  const building = bigObstacles[id][0];
  const scale = bigObstacles[id][1];
  const theta_y = bigObstacles[id][2];

  if(checkGenerationCollision(x,y,z)){
    WORLD.add3DObject(
      scene,
      model.getObjectByName("bigObs_"+building),
      building,
      path + building + '.mtl',
      path + building + '.obj',
      [x, y, z],
      [theta, theta_y, 0.0],
      scale,
      false,
      true,
      obstacle_type,
      0,
      -1,
    );

    for(let i=-space_bigObstacles[id]; i<=space_bigObstacles[id]; i++){
      for(let j=-1.5; j<=1.5; j++){
        for(let k=-1; k<=1; k++){
          positionOfBigObstacles.push({x:x+i, y:y+j, z:z+k});
        }
      }
    }
  }
}

function placeRoad(scene, model, deg, level, orientationVehicles, ice_bool, trainOrIceLevel2=false, index=-1) {
  const r = 100;
  const radians = 57.2958;
  let theta; 
  let y; 
  let z;

  let path;
  let scale;
  let axis_z;
  let ice = -1;
  if (level==1){
    path = 'assets/models/level1/road/road_crossing'
    scale = [5.0, 5.0, 5.0];

  } else if (level == 2) {
    if(ice_bool){ // if I put the Iced lake
      path = 'assets/models/level2/road/block_ice'
      scale = [2.9, 0.5, 6.8];
      deg += 3.0; // aligning the roads
      if (index == 1 || index == 40 || index == 26 || index == 12) ice = 1;
      else if (index == 4 || index == 44 || index == 31 || index == 18) ice = 0;
    } else { 
      if(trainOrIceLevel2){
        path = 'assets/models/level2/road/trackStraight'
        scale = [16.0, 10.0, 10.0];
        deg -= 0.65; // aligning the roads
      }else{
        path = 'assets/models/level2/road/SnowTerrain'
        scale = [0.12, 0.01, 0.05];
      }
    }
  } else if (level == 3) {
    path = 'assets/models/level3/road/pathCobbled'
    scale = [2.4, 2.4, 2.4];
  }

  theta = deg / radians;
  if(ice_bool) y = (r-0.5)*Math.cos(theta);
  else y = r*Math.cos(theta);
  z = r*Math.sin(theta);

  const h = 6;
  for (var i=-h; i<h; i++) {
    let x;
    if(level==3) x=5*i+2;
    else x = 5*i+2.5;
    
    WORLD.add3DObject(
      scene,
      model.getObjectByName("roadsModel"),
      'road',
      path + '.mtl',
      path + '.obj',
      [x, y, z],
      [theta, 0, 0],
      scale,
      false,
      true,
      0,
      0,
      ice
    );
    positionOfRoads.push({y:y, z:z});
  }

  if(!ice_bool){
    placeRandomVehicle(scene, model.getObjectByName("vehiclesModel"), deg, orientationVehicles, level, 2, trainOrIceLevel2);
  }
}

function placeRandomVehicle(scene, model, deg, orientation, level, numberOfCycles, trainOrIceLevel2) {
  let path;
  let vehicles;
  let dy;
  let valueForCheckingCollisionsVehicle = 3;
  let offsetSnow = 0;
  
  if(level==1){
    path = 'assets/models/level1/vehicles/';
    dy = 0;

    vehicles = [
      ['ambulance', [1.5, 1.5, 1.5], orientation*pi/2.0],
      ['delivery', [1.5, 1.5, 1.5], orientation*pi/2.0],
      ['deliveryFlat', [1.5, 1.5, 1.5], orientation*pi/2.0],
      ['firetruck', [1.5, 1.5, 1.5], orientation*pi/2.0],
      ['garbageTruck', [1.5, 1.5, 1.5], orientation*pi/2.0],
      ['hatchbackSports', [1.5, 1.5, 1.5], orientation*pi/2.0],
      ['police', [1.5, 1.5, 1.5], orientation*pi/2.0],
      ['race', [1.5, 1.5, 1.5], orientation*pi/2.0],
      ['raceFuture', [1.5, 1.5, 1.5], orientation*pi/2.0],
      ['sedan', [1.5, 1.5, 1.5], orientation*pi/2.0],
      ['sedanSports', [1.5, 1.5, 1.5], orientation*pi/2.0],
      ['suv', [1.5, 1.5, 1.5], orientation*pi/2.0],
      ['suvLuxury', [1.5, 1.5, 1.5], orientation*pi/2.0],
      ['taxi', [1.5, 1.5, 1.5], orientation*pi/2.0],
      ['tractor', [1.5, 1.5, 1.5], orientation*pi/2.0],
      ['tractorPolice', [1.5, 1.5, 1.5], orientation*pi/2.0],
      ['tractorShovel', [1.5, 1.5, 1.5], orientation*pi/2.0],
      ['truck', [1.5, 1.5, 1.5], orientation*pi/2.0],
      ['truckFlat', [1.5, 1.5, 1.5], orientation*pi/2.0],
      ['van', [1.5, 1.5, 1.5], orientation*pi/2.0]
    ];
  } else if(level==2){
    path = 'assets/models/level2/vehicles/';
    dy = 0.4;
    if(trainOrIceLevel2){
      deg += 0.65; // aligning the vehicles
      vehicles = [
        ['trainLoc', [14, 10, 7], orientation*pi]
      ];
      orientation=-orientation;
      valueForCheckingCollisionsVehicle = 8;
    } else {
      vehicles = [
        ['snowman', [5, 5, 5], -orientation*pi/2.0],
        ['snowmanFancy', [5, 5, 5], -orientation*pi/2.0]
      ];
      offsetSnow=0.01;
    }

  } else if (level == 3) {
    path = 'assets/models/level3/monsters/';

    dy = 0;
    let angle = 0;
    if (orientation == -1) angle = pi;

    vehicles = [
      ['Slime', [1.5, 1.5, 1.5], angle]
    ];
  }

  const r = 100;
  const radians = 57.2958;
  const theta = deg / radians;
  const y = r * Math.cos(theta+offsetSnow);
  const z = r * Math.sin(theta);

  let x;
  let archieved_x = [];

  for(var i=0; i<numberOfCycles; i++){ // Generating a number numberOfCycles of machines
    do{
      x = getRandomInt(-20, 20);
    }while(archieved_x.includes(x));
    
    for(var j=-valueForCheckingCollisionsVehicle; j<=valueForCheckingCollisionsVehicle; j++){
      archieved_x.push(x+j);
    }

    const nVehicles = vehicles.length;
    const id = getRandomInt(0, nVehicles-1);
    const vehicle = vehicles[id][0];
    const scale = vehicles[id][1];
    const theta_y_final = vehicles[id][2];
  
    
    WORLD.add3DObject(
      scene,
      model.getObjectByName("vehicle_"+vehicle),
      vehicle,
      path + vehicle + '.mtl',
      path + vehicle + '.obj',
      [x, y, z],
      [theta, theta_y_final, 0.0],
      scale,
      false,
      true,
      0,
      orientation,
      -1
    );
  
  }
}

function placeCoin(scene, model, deg) {
  const r = 100;
  const radians = 57.2958;
  let x = getRandomInt(-23, 23);
  let theta = deg / radians;
  let y = r * Math.cos(theta);
  let z = r * Math.sin(theta);

  let path, scale;

  while (!checkGenerationCollision(x, y, z)) x = getRandomInt(-24, 24);

  path = 'assets/models/coin/';

  scale = [3.0, 3.0, 3.0];

  // left wall
  WORLD.add3DObject(
    scene,
    model.getObjectByName("coinsModel"),
    'coin',
    path + 'coin.mtl',
    path + 'coin.obj',
    [x, y, z],
    [theta, 0.0, 0.0],
    scale,
    false,
    true,
    4,
    0,
    -1
  );
}

export function generate_new_coin(scene, model) {
  // initially we generate a new coin when we collect one, but then in order to not have
  // a huge array, we simply make the ones we had generated before visible again
  if (WORLD.coinsCollisions.length < 10) {
    let deg = getRandomInt(0, 27);
    placeCoin(scene, model, deg);
  } else {
    for (let coin of WORLD.coinsCollisions) {
      if (!coin.visible) {
        coin.visible = true;
        break;
      }
    }
  }
}