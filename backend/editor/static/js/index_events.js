var editor;
var current_file = "";
var app_name = "default"

function getCookie(name) 
{
    let cookieValue = null;
    if (document.cookie && document.cookie !== '') {
        const cookies = document.cookie.split(';');
        for (let i = 0; i < cookies.length; i++) {
            const cookie = cookies[i].trim();
            // Does this cookie string begin with the name we want?
            if (cookie.substring(0, name.length + 1) === (name + '=')) {
                cookieValue = decodeURIComponent(cookie.substring(name.length + 1));
                break;
            }
        }
    }
    return cookieValue;
}

// Function to setup the ACE editor
function initEditor()
{
    editor = ace.edit("editor");
    editor.setTheme("ace/theme/monokai");
    editor.session.setMode("ace/mode/python");
}

// Populate the file list with updated data
function update_file_list() 
{
    // Make a GET request to Django API to get the list of files
    fetch('/tree_api/get_file_list/')
    .then(response => response.json())
    .then(data => {
        // Clear the existing file list
        const fileList = document.getElementById('file-list');
        while (fileList.firstChild) {
            fileList.removeChild(fileList.firstChild);
        }
        
        // Add new list items based on the received list of files
        if (data.file_list && data.file_list.length > 0) {
            data.file_list.forEach(filename => {

                // Create a new li
                const newFileItem = document.createElement('li');
                newFileItem.textContent = filename;

                // Add the click callback
                newFileItem.addEventListener("click", function() {
                    readFile(this.textContent);
                })
                fileList.appendChild(newFileItem);
            });
        }
    })
    .catch(error => console.error('Error:', error));
}

// Function to read file content
function readFile(filename) 
{
    fetch(`/tree_api/get_file/?filename=${filename}`)
        .then(response => response.json())
        .then(data => {
            if (data.content !== undefined) {
                editor.setValue(data.content);
            }
        })
        .catch(error => {
            console.error("Error reading file:", error);
        });
    
    current_file = filename;
    const current_file_text = document.getElementById('current-file-text');
    current_file_text.textContent = "Selected file: " + current_file;
}

// Create a new file, with the input name from the user
function createFile()
{
    let filename = prompt("Enter new file name:", "new_file.txt");
    if (filename) {
        fetch('/tree_api/create_file/', {
            method: 'POST',
            headers: {
                'Content-Type': 'application/json',
                'X-CSRFToken': getCookie('csrftoken')
            },
            body: JSON.stringify({ 'filename': filename })
        })
        .then(response => response.json())
        .then(data => {
            if (data.success) {
                update_file_list();
                readFile(filename);
            }
        })
        .catch(error => console.error('Error:', error));
    } 
}

// Delete selected file
function deleteFile()
{
    filename = current_file;
    if (filename) {
        fetch('/tree_api/delete_file/', {
            method: 'POST',
            headers: {
                'Content-Type': 'application/json',
                'X-CSRFToken': getCookie('csrftoken')
            },
            body: JSON.stringify({ 'filename': filename })
        })
        .then(response => response.json())
        .then(data => {
            if (data.success) {
                update_file_list();
            }
        })
        .catch(error => console.error('Error:', error));
    } 
}

// Save the content of the current file
function saveFile() 
{
    const filename = current_file;
    const content = editor.getValue();

    fetch('/tree_api/save_file/', {
        method: 'POST',
        headers: {
            'Content-Type': 'application/json',
            'X-CSRFToken': getCookie('csrftoken')
        },
        body: JSON.stringify({ 'filename': filename, 'content': content })
    })
    .then(response => response.json())
    .then(data => {
        if (data.success) {
            console.log("File saved successfully");
        } else {
            alert("Error saving file");
        }
    })
    .catch(error => console.error('Error:', error));
}

// Function to download app
function downloadApp() 
{
    // Open a new window or tab and let the browser handle the download
    app_endpoint = '/tree_api/download_app/?app_name=' + app_name;
    window.open(app_endpoint, '_blank');
}

document.addEventListener("DOMContentLoaded", function() 
{ 
    // Init the editor
    initEditor();

    // Populate the file list
    update_file_list();

    // Event listener for "Create New File" button
    document.getElementById("create-file-btn").addEventListener("click", createFile);

    // Event listener for "Delete Current File" button
    document.getElementById("delete-file-btn").addEventListener("click", deleteFile);

    // Event listener for "Save file" button
    document.getElementById("save-file-btn").addEventListener("click", saveFile);

    // Event listener for "Save file" button
    document.getElementById("download-app-btn").addEventListener("click", downloadApp);
});