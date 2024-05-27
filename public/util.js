export const download = (obj) => {
    const blob = new Blob([JSON.stringify(obj, null, "    ")],  {type: "application\/json"});
    const link = document.createElement("a");
    document.body.appendChild(link);
    link.href = URL.createObjectURL(blob);
    link.download = "boidTestJs.json";
    link.click();
    document.body.removeChild(link);
}