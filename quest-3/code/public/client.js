console.log("clientside js running");

const button = document.querySelector("button");

button.addEventListener("click", (e) => {
  e.preventDefault();
  //console.log("BUTTON PRESSED");

  fetch("http://rettopyrrah.ddns.net:1130/button").then((response)=>{
    response.json().then((data) => {
      console.log(data.message);
    })
  });
});
