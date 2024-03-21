const rad = function (x: number) {
  return (x * Math.PI) / 180;
};

export const distance = (prevLat: number, prevLong: number, newLat: number, newLong: number) => {
  const R = 6378.137; //지구 반지름
  const dLat = Math.abs(rad(newLat - prevLat));
  const dLong = Math.abs(rad(newLong - prevLong));

  const a = Math.abs(
      Math.sin(dLat / 2) * Math.sin(dLat / 2) +
          Math.cos(rad(prevLat)) * Math.cos(rad(newLat)) * Math.sin(dLong / 2) * Math.sin(dLong / 2),
  );
  const c = 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1 - a));
  const d = R * c;
  
  return d;
};
