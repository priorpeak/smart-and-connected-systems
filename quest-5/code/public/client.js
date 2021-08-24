console.log("clientside js running");

const button = document.querySelector("#onButton");

// const button2 = document.querySelector("#offButton");

button.addEventListener("click", (e) => {
  e.preventDefault();
  console.log("TOGGLE BUTTON PRESSED");

  fetch("button").then((response) => {
    // response.json().then((data) => {
    //   console.log(data.message);
    // })
    console.log("MADE IT HERE \n");
  });
});

// button2.addEventListener("click", (e) => {
//   e.preventDefault();
//   console.log("OFF BUTTON PRESSED");

//   // fetch("http://rettopyrrah.ddns.net:1130/button").then((response) => {
//   //   response.json().then((data) => {
//   //     console.log(data.message);
//   //   })
//   // });
// });