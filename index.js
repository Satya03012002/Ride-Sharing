import express from "express";
import bodyParser from "body-parser";
import "./db/dbConn.js";
import cors from "cors";
import SignInRouter from "./router/loginRequest/router.js";
import dotenv from "dotenv";
import fs from "fs";

dotenv.config({ silent: process.env.NODE_ENV === "production" });
const app = express();
const PORT = process.env.PORT || 5000;
app.use(cors());
app.use(bodyParser.json({ limit: "50mb" }));
app.use(bodyParser.urlencoded({ limit: "50mb", extended: true }));
//app.use("/signin",SignInRouter);
import { exec, spawn } from "child_process";
import axios from "axios";

const get_data_origin = async (lat, lng) => {
  console.log("-----------origin data");
  console.log(lat, lng);

  const res = await axios.get(
    `https://maps.googleapis.com/maps/api/place/search/json?location=${lat},${lng}&radius=200000&types=hospital_or_supermarket&sensor=true&key=""`
  );

  // console.log(res)

  return res.data;
};
const get_data_dest = async (lat, lng) => {
  console.log("-----------destination data");

  console.log(lat, lng);

  const res = await axios.get(
    `https://maps.googleapis.com/maps/api/place/search/json?location=${lat},${lng}&radius=200000&types=hospital_or_supermarket&sensor=true&key=""`
  );
  //console.log(res)

  return res.data;
};

app.post("/runcpp", async (req, res) => {
  console.log("-----------req.body", req.body);

  let temp = req.body;

  if(temp.start_location.lat && temp.start_location.lng && temp.end_location.lat && temp.end_location.lng){

    if (temp.start_location.lat && temp.start_location.lng) {
      console.log(temp.start_location.lat, temp.start_location.lng);
      let origin_store = await get_data_origin(
        temp.start_location.lat,
        temp.start_location.lng
      );
      console.log(origin_store);
      temp.origin_store = origin_store.results;
    }
  
    if (temp.end_location.lat && temp.end_location.lng) {
      let dest_store = await get_data_dest(
        temp.end_location.lat,
        temp.end_location.lng
      );
      temp.dest_store = dest_store.results;
      console.log(dest_store);
    }

    let mid_store = await get_data_dest(
      (temp.end_location.lat +  temp.start_location.lat)/2,
      (temp.end_location.lng + temp.start_location.lng)/2
    );
    temp.mid_store = mid_store.results;

    
    let mid_store_2 = await get_data_dest(
      (temp.end_location.lat+(temp.end_location.lat +  temp.start_location.lat)/2)/2,
      (temp.end_location.lng+(temp.end_location.lng + temp.start_location.lng)/2)/2
    );
    temp.mid_store_2 = mid_store_2.results;
        
    let mid_store_3 = await get_data_dest(
      (temp.start_location.lat+(temp.end_location.lat +  temp.start_location.lat)/2)/2,
      (temp.start_location.lng+(temp.end_location.lng + temp.start_location.lng)/2)/2
    );
    temp.mid_store_3 = mid_store_3.results;

  }



  console.log("------------temp", temp);

  let stores = [];
  let count = 0


  if (temp && temp.origin_store && temp.dest_store) {
    temp.origin_store.map((geo) => {
      let obj = {};
      obj.title = geo.name;
      obj.position = geo.geometry.location;
     
      if(count < 3){
        stores.push(obj);
        count++

      }
    });
    temp.mid_store_3.map((geo) => {
      let obj = {};
      obj.title = geo.name;
      obj.position = geo.geometry.location;
      if(count < 7){
        stores.push(obj);
        count++
      }
    });
  
    temp.mid_store.map((geo) => {
      let obj = {};
      obj.title = geo.name;
      obj.position = geo.geometry.location;
      if(count < 12){
        stores.push(obj);
        count++
      }
    });
    temp.mid_store_2.map((geo) => {
      let obj = {};
      obj.title = geo.name;
      obj.position = geo.geometry.location;
      if(count < 16){
        stores.push(obj);
        count++
      }
    });
    temp.dest_store.map((geo) => {
      let obj = {};
      obj.title = geo.name;
      obj.position = geo.geometry.location;
      if(count < 22){
        stores.push(obj);
        count++
      }
    });

    let obj_src = {};
    obj_src.title = temp.start_address;
    obj_src.position = temp.start_location;
    stores.unshift(obj_src);

    let obj_dest = {};
    obj_dest.title = temp.end_address;
    obj_dest.position = temp.end_location;
    stores.push(obj_dest);
  }

  for (let i = 0; i < stores.length; i++) {
    stores[i].no = i;
    stores[i].hours = "9AM to 9PM";
  }

  for (let i = 0; i < stores.length; i++) {
    stores[i].position.lat = parseFloat(stores[i].position.lat.toFixed(4));
    stores[i].position.lng = parseFloat(stores[i].position.lng.toFixed(4));
  }
  let path = "";
  console.log("------------------store");
  console.log(stores);
  let outputData = "";
  let arr = [];

  // Save the C++ code to a temporary file
  const cppFilePath = "./main.cpp";
  const compiledExePath = "./hello"; // Output executable path

  const compileCommand = `g++ ${cppFilePath} -o ${compiledExePath}`;

 let data =  await new Promise((resolve, reject) => {
    exec(compileCommand, async (compileError, compileStdout, compileStderr) => {
      if (compileError) {
        console.error(`Error compiling C++ code: ${compileError}`);
        return;
      }
  
      // Run the compiled executable
      const runCommand = `./${compiledExePath}`;
      const cppProcess = spawn(runCommand, { stdio: ["pipe", "pipe", "pipe"] });
  
      // Send input to the C++ program
      const inputData = JSON.stringify(stores);
      cppProcess.stdin.write(inputData);
      cppProcess.stdin.end();
  
      // Capture output from the C++ program
      let outputData = "";
      cppProcess.stdout.on("data", (data) => {
        outputData += data.toString();
        arr.push(outputData);
       console.log("----------------outputdata", outputData);
        //const filePath = "input.json";
        //fs.writeFileSync(filePath, JSON.stringify(outputData));
  
        //const readData = fs.readFileSync("./input.json", "utf-8");
        //const parsedData = JSON.parse(readData);
  
       // console.log("Read Data:", parsedData);
      });
  
      // Handle errors and completion of the C++ program
      cppProcess.on("error", (error) => {
        console.error(`Error running compiled C++ code: ${error}`);
      });
  
      cppProcess.on("close", (code) => {
        if (code === 0) {
          // console.log(`C++ Program Output:\n${outputData}`);
          resolve(outputData)
          return;
        } else {
          console.error(`C++ Program exited with code ${code}`);
        }
      });
    });
  });


  console.log("----------------outputdata", data);
  let myArray = data.split("--");
  console.log(myArray)
  if(myArray && myArray.length > 0){
    myArray =  myArray.map((arr)=> {return parseInt(arr)} )
  }
  console.log(myArray)

  let datapath = {
    stores:stores,
    path :myArray
  }



  return res.status(200).json({datapath});
});



// for (let i = 0; i < stores.length; i++) {
//   stores[i].no = i;
// }

console.log("inside---------------------------");
//console.log(stores)

// Save the C++ code to a temporary file
// const cppFilePath = './main.cpp';
// const compiledExePath = './hello'; // Output executable path

// const compileCommand = `g++ ${cppFilePath} -o ${compiledExePath}`;
// exec(compileCommand, (compileError, compileStdout, compileStderr) => {
//     if (compileError) {
//         console.error(`Error compiling C++ code: ${compileError}`);
//         return;
//     }

//     // Run the compiled executable
//     const runCommand = `./${compiledExePath}`;
//     const cppProcess = spawn(runCommand, { stdio: ['pipe', 'pipe', 'pipe'] });

//     // Send input to the C++ program
//     const inputData = JSON.stringify(stores);
//     cppProcess.stdin.write(inputData);
//     cppProcess.stdin.end();

//     // Capture output from the C++ program
//     let outputData = '';
//     cppProcess.stdout.on('data', (data) => {
//         outputData += data.toString();
//         console.log("----------------outputdata",outputData)
//     });

//     // Handle errors and completion of the C++ program
//     cppProcess.on('error', (error) => {
//         console.error(`Error running compiled C++ code: ${error}`);
//     });

//     cppProcess.on('close', (code) => {
//         if (code === 0) {
//             console.log(`C++ Program Output:\n${outputData}`);
//         } else {
//             console.error(`C++ Program exited with code ${code}`);
//         }
//     });
// });

app.listen(PORT, () => {
  //console.log(`server running successfully on PORT : ${PORT}`)
});

function haversine(lat1, lon1, lat2, lon2) {
  const R = 6371; // Earth's radius in kilometers

  // Convert latitude and longitude from degrees to radians
  const toRadians = (angle) => angle * (Math.PI / 180);
  lat1 = toRadians(lat1);
  lon1 = toRadians(lon1);
  lat2 = toRadians(lat2);
  lon2 = toRadians(lon2);

  // Differences in coordinates
  const dlat = lat2 - lat1;
  const dlon = lon2 - lon1;

  // Haversine formula
  const a =
    Math.sin(dlat / 2) ** 2 +
    Math.cos(lat1) * Math.cos(lat2) * Math.sin(dlon / 2) ** 2;
  const c = 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1 - a));
  const distance = R * c;

  return distance;
}
var stores = [
  {
    title: "Store 1",
    position: { lat: 28.6339, lng: 77.5177 },
    hours: "8AM to 10PM",
  },
  {
    title: "Store 2",
    position: { lat: 28.6804, lng: 77.6177 },
    hours: "9AM to 9PM",
  },
  {
    title: "Store 3",
    position: { lat: 28.6104, lng: 77.3277 },
    hours: "8AM to 10PM",
  },
  {
    title: "Store 4",
    position: { lat: 28.6704, lng: 77.5377 },
    hours: "9AM to 9PM",
  },
  {
    title: "Store 5",
    position: { lat: 28.6204, lng: 77.2977 },
    hours: "8AM to 10PM",
  },
  {
    title: "Store 6",
    position: { lat: 28.7764, lng: 77.4777 },
    hours: "9AM to 9PM",
  },
  {
    title: "Store 7",
    position: { lat: 28.6874, lng: 77.6377 },
    hours: "9AM to 9PM",
  },
  {
    title: "Store 8",
    position: { lat: 28.6144, lng: 77.6977 },
    hours: "8AM to 10PM",
  },
  {
    title: "Store 9",
    position: { lat: 28.6724, lng: 77.5977 },
    hours: "9AM to 9PM",
  },
  {
    title: "Store 10",
    position: { lat: 28.6214, lng: 77.3977 },
    hours: "8AM to 10PM",
  },
  {
    title: "Store 11",
    position: { lat: 28.7694, lng: 77.5777 },
    hours: "9AM to 9PM",
  },
  {
    title: "Store 12",
    position: { lat: 28.6692, lng: 77.4538 },
    hours: "8AM to 10PM",
  },
  {
    title: "Store 13",
    position: { lat: 28.6634, lng: 77.6127 },
    hours: "9AM to 9PM",
  },
  {
    title: "Store 14",
    position: { lat: 28.6544, lng: 77.6277 },
    hours: "8AM to 10PM",
  },
  {
    title: "Store 15",
    position: { lat: 28.67454, lng: 77.7377 },
    hours: "9AM to 9PM",
  },
  // {
  //   title: 'Store 16',
  //   position: { lat: 28.6104, lng:  77.4977 },
  //   hours: '8AM to 10PM'
  // },
  // {
  //   title: 'Store 17',
  //   position: { lat: 28.7714, lng:  77.4977},
  //   hours: '9AM to 9PM'
  // },
  // {
  //   title: 'Store 18',
  //   position: { lat: 28.6374, lng:  77.6977},
  //   hours: '9AM to 9PM'
  // },
  // {
  //   title: 'Store 19',
  //   position: { lat: 28.6444, lng:  77.3977 },
  //   hours: '8AM to 10PM'
  // },
  // {
  //   title: 'Store 20',
  //   position: { lat: 28.6724, lng:  77.5977},
  //   hours: '9AM to 9PM'
  // },
  // {
  //   title: 'Store 21',
  //   position: { lat: 28.4214, lng:  77.5977 },
  //   hours: '8AM to 10PM'
  // },
  // {
  //   title: 'Store 22',
  //   position: { lat: 28.4694, lng:  77.8777},
  //   hours: '9AM to 9PM'
  // }
];