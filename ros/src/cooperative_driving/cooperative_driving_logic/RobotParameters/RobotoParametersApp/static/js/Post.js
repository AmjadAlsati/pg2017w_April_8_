        function ChangeParm(clicked_id){
            var ParameterName = document.getElementById(clicked_id).id; //get the name of clicked Parameter
            var ParameterValue=document.getElementsByName(clicked_id)[0].value;//get the value of clicked Parameter
             
            console.log("a function is getting called");


            var JsonObject = {};// create a JSON object
            JsonObject[clicked_id] = ParameterValue; //fill the object with the name and value
            

        $.ajax({


            type: 'POST',
            url: '/post/',
            data:{

                  JsonObject,
                  csrfmiddlewaretoken: getCSRFTokenValue()


            },
            sucess:function()
            {

                alert("success");
            }
        });

        }


