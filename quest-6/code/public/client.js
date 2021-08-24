console.log("clientside js running");

const button = document.querySelector("#catButton");
const button2 = document.querySelector("#dogButton");

button.addEventListener("click", (e) => {
  e.preventDefault();
  console.log("CAT BUTTON PRESSED");

  fetch("catButton").then((response) => {
    // response.json().then((data) => {
    // console.log(data.message);
    // })
    console.log("catButton pressed \n");
  });
});

button2.addEventListener("click", (e) => {
  e.preventDefault();
  console.log("DOG BUTTON PRESSED");

  fetch("dogButton").then((response) => {
    // response.json().then((data) => {
    // console.log(data.message);
    // })
    console.log("dogButton pressed \n");
  });
});
