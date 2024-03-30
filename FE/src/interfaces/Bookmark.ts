export interface BookmarkInputType {
	name: string;
	latitude: number;
	longitude: number;
	address: string;
}

export interface BookmarkType {
	bookmarkId : number
	name : string
	place : PlaceType 
}

export interface PlaceType {
	id : number
	latitude : number
	longitude : number
	address : string
}
