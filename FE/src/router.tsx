import { createBrowserRouter } from 'react-router-dom';
import { LoginPage } from './pages/login/LoginPage.tsx';
import { SignUpPage } from './pages/signup/SignUpPage.tsx';
import { MainPage } from './pages/main/MainPage.tsx';
import { Reservation } from './pages/reservation/Reservation.tsx';
import { NewReservation } from './pages/reservation/NewReservation.tsx';
import { TogetherRsv } from './pages/reservation/TogetherRsv.tsx';

import { History } from './pages/history/History.tsx';
import { NotFound } from './pages/NotFound.tsx';
import { UserPage } from './pages/profile/UserPage.tsx';
import { SetDeparture } from './pages/main/components/SetDeparture.tsx';
import { SetTogetherDeparture } from './pages/main/components/SetTogetherDeparture.tsx';
import { SetArrival } from './pages/main/components/SetArrival.tsx';
import { SetDepartureByMap } from './pages/main/components/SetDepartureByMap.tsx';
import { SetArrivalByMap } from './pages/main/components/SetArrivalByMap.tsx';
import { Favorite } from './pages/favorite/Favorite.tsx';
import PageWithLogin from './pages/PageWithLogin.tsx';
import PageWithoutLogin from './pages/PageWithoutLogin.tsx';
import { CurLocCar } from './pages/reservation/components/CurLocCar.tsx';

export interface IRouterItem {
	path: string;
	element: JSX.Element;
	withAuth: boolean;
}

export const rounterItems: IRouterItem[] = [
	{ path: '/', element: <LoginPage />, withAuth: false },
	{ path: '/signup', element: <SignUpPage />, withAuth: false },
	{ path: '/main', element: <MainPage />, withAuth: true },
	{ path: '/reservation', element: <Reservation />, withAuth: true },
	{ path: '/reservation/new', element: <NewReservation />, withAuth: true },
	{ path: '/reservation/:rsvId', element: <TogetherRsv />, withAuth: true },
	{ path: '/setDeparture', element: <SetDeparture />, withAuth: true },
	{ path: '/setTogetherDeparture/:rsvId', element: <SetTogetherDeparture />, withAuth: true },
	{ path: '/setArrival', element: <SetArrival />, withAuth: true },
	{ path: '/setDepartureByMap', element: <SetDepartureByMap />, withAuth: true },
	{ path: '/setArrivalByMap', element: <SetArrivalByMap />, withAuth: true },
	{ path: '/history', element: <History />, withAuth: true },
	{ path: '/move', element: <CurLocCar />, withAuth: true },
	{ path: '/favorite', element: <Favorite />, withAuth: true },
	{ path: '/profile', element: <UserPage />, withAuth: true },
	{ path: '/*', element: <NotFound />, withAuth: false },
];

export const router = createBrowserRouter(
	rounterItems.map((routerItem: IRouterItem) => {
		return routerItem.withAuth
			? { path: routerItem.path, element: <PageWithLogin>{routerItem.element}</PageWithLogin> }
			: { path: routerItem.path, element: <PageWithoutLogin> {routerItem.element}</PageWithoutLogin> };
	}),
);
