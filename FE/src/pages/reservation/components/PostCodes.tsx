import DaumPostcodeEmbed from 'react-daum-postcode';

export const Postcode = () => {
	const handleComplete = (data: any) => {
		let fullAddress = data.address;
		let extraAddress = '';
		if (data.addressType === 'R') {
			if (data.bname !== '') {
				extraAddress += data.bname;
			}
			if (data.buildingName !== '') {
				extraAddress += extraAddress !== '' ? `, ${data.buildingName}` : data.buildingName;
			}
			fullAddress += extraAddress !== '' ? ` (${extraAddress})` : '';
		}
		console.log(fullAddress); // e.g. '서울 성동구 왕십리로2길 20 (성수동1가)'
	};
	return <DaumPostcodeEmbed onComplete={handleComplete} />;
};

// import React, { useEffect, useState } from "react";
// import { useDaumPostcodePopup } from "react-daum-postcode";
// import "./PostCodes.css";

// const PostCode = (props) => {
//   const scriptUrl =
//     "https://t1.daumcdn.net/mapjsapi/bundle/postcode/prod/postcode.v2.js";
//   const open = useDaumPostcodePopup(scriptUrl);
//   let [roadAdd, setRoadAdd] = useState(props.selectAddress.roadAddress);
//   let [buildName, setBuildName] = useState(props.selectAddress.buildingName);

//   const handleComplete = (data) => {
//     if (data.addressType === "R") {
//       setRoadAdd(data.roadAddress);
//       if (data.buildingName !== "") {
//         setBuildName(data.buildingName);
//       } else {
//         setBuildName("건물 명");
//       }
//     }
//   };

//   useEffect(() => {
//     props.setAddress({
//       roadAddress: roadAdd,
//       buildingName: buildName,
//     });
//   }, [roadAdd, buildName]);

//   const handleClick = () => {
//     open({ onComplete: handleComplete });
//   };

//   return (
//     <div className='post-code-i' onClick={handleClick}>
//       <div className='road-address-i'>{roadAdd}</div>
//       <div className='building-name-i'>{buildName}</div>
//     </div>
//   );
// };

// export default PostCode;
